package frc.robot.commands;

import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.wpilibj2.command.Commands.run;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotState;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.hood.HoodConstants;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterConstants;
import frc.robot.subsystems.turret.Turret;
import frc.robot.subsystems.turret.TurretConstants;
import frc.robot.util.KinematicsHelper;
import frc.robot.util.ShotWarmstartTable;
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

/** Auto-aim: hub shots (funnel solver) or pass shots (drag solver). */
public final class AutoAim {

  private static final double kFieldLengthMeters = 16.54;
  private static final double kAllianceLineXBlue = 4.5;
  private static final double kLaunchOffsetMeters = 0.28;
  private static final double kGravity = 9.81;
  private static final double kTrajectoryDt = 0.02;
  private static final int kTrajectoryMaxPts = 128;
  private static final int kLeadIterations = 3;
  private static final double kSidePassRPMDefault = 2500.0;
  private static final double kConvergeThresholdDist = 0.01;
  private static final double kStationarySpeedThreshold = 0.1;
  private static final double kPoseLatencyMaxSec = 0.1;
  private static final double kAimFilterAlpha = 0.3;
  private static final int kLogInterval = 3;

  private static double filteredAimAngle = 0.0;
  private static int logCounter = 0;

  private AutoAim() {}

  /** One cycle: set shooter RPM, hood angle, turret heading. Default: turret in [-180°, 180°]. */
  public static void updateAutoAim(Turret turret, Shooter shooter, Hood hood, boolean hubMode) {
    updateAutoAim(turret, shooter, hood, hubMode, false);
  }

  /** allowFullRange true = use full 720° while shooting; false = stay in [-180°, 180°] */
  public static void updateAutoAim(
      Turret turret, Shooter shooter, Hood hood, boolean hubMode, boolean allowFullRange) {
    RobotState state = RobotState.getInstance();
    var latestPose = state.getLatestFieldToRobot();
    if (latestPose == null) {
      state.setAutoAimArcValid(false);
      return;
    }

    Pose2d robotPose = latestPose.getValue();

    var fieldSpeeds = state.getLatestMeasuredFieldRelativeChassisSpeeds();
    double robotVx = fieldSpeeds != null ? fieldSpeeds.vxMetersPerSecond : 0.0;
    double robotVy = fieldSpeeds != null ? fieldSpeeds.vyMetersPerSecond : 0.0;
    double robotOmega = fieldSpeeds != null ? fieldSpeeds.omegaRadiansPerSecond : 0.0;
    double headingRate = state.getHeadingRateRadPerSec();

    // Pose latency compensation: propagate pose forward by its age
    double poseTimestamp = latestPose.getKey();
    double poseAgeSec = Timer.getFPGATimestamp() - poseTimestamp;
    if (poseAgeSec > 0.0 && poseAgeSec < kPoseLatencyMaxSec) {
      Translation2d propagated =
          robotPose
              .getTranslation()
              .plus(new Translation2d(robotVx * poseAgeSec, robotVy * poseAgeSec));
      robotPose = new Pose2d(propagated, robotPose.getRotation());
    }

    boolean pastAllianceLine = isPastAllianceLine(robotPose, state.isRedAlliance());
    boolean effectiveHubMode = hubMode && !pastAllianceLine;
    Translation3d staticTarget = resolveTarget(robotPose, state, effectiveHubMode);

    Translation2d turretPivotField = KinematicsHelper.getTurretPivotTranslation(robotPose);
    double staticDistM = turretPivotField.getDistance(staticTarget.toTranslation2d());

    double velConstant = getLaunchVelConstant();
    double launchHeight = Constants.hoodBasePose.getZ();

    Translation2d pivotOffset = turretPivotField.minus(robotPose.getTranslation());
    double pivotVx = robotVx - headingRate * pivotOffset.getY();
    double pivotVy = robotVy + headingRate * pivotOffset.getX();

    boolean isStationary =
        Math.abs(robotVx) < kStationarySpeedThreshold
            && Math.abs(robotVy) < kStationarySpeedThreshold
            && Math.abs(robotOmega) < kStationarySpeedThreshold;

    double maxHoodRad = HoodConstants.kMaxAngle.in(Radians);

    Translation3d aimTarget = staticTarget;
    double hoodAngleRad = 0.0;
    double tof = 0.0;
    double launchSpeedMps = 5.0;
    double lastDistM = 0.0;
    double lastFunnelRadiusM = 0.0;
    double lastFunnelClearHeightM = 0.0;
    double lastEffectiveDistFromLaunchM = 0.0;
    boolean lastFunnelConstraintMet = true;

    int leadSolveIterations = 0;
    boolean leadConvergedEarly = false;
    int firstFunnelValidIteration = -1;

    if (isStationary) {
      // Fast path: no motion compensation needed
      double dist = turretPivotField.getDistance(aimTarget.toTranslation2d());

      if (effectiveHubMode) {
        leadSolveIterations = 1;
        lastDistM = dist;
        lastFunnelRadiusM = Constants.kHubFunnelRadiusMeters;
        lastFunnelClearHeightM =
            Constants.kHubFunnelHeightMeters + Constants.kHubFunnelClearanceMeters;
        double fallbackSpeedMps = (ShooterConstants.kAutoAimRPM / 60.0) * velConstant;
        double distFromLaunch = Math.max(0.01, dist - kLaunchOffsetMeters);

        var solved =
            KinematicsHelper.solveFunnelClearance(
                distFromLaunch,
                launchHeight,
                aimTarget.getZ(),
                lastFunnelRadiusM,
                lastFunnelClearHeightM,
                fallbackSpeedMps);
        hoodAngleRad = solved.hoodAngleRad();
        launchSpeedMps = solved.launchSpeedMps();
        tof = solved.timeOfFlightSec();
        lastEffectiveDistFromLaunchM = solved.effectiveDistM();
        lastFunnelClearHeightM = solved.effectiveFunnelClearHeightM();
        lastFunnelConstraintMet = solved.funnelConstraintMet();
        if (lastFunnelConstraintMet) {
          firstFunnelValidIteration = 0;
        }
      } else {
        leadSolveIterations = 1;
        double angleRad = isPassCenter() ? maxHoodRad : HoodConstants.kMinAngle.in(Radians);
        double[] arc =
            KinematicsHelper.solveForTargetHighArc(dist, launchHeight, aimTarget.getZ(), angleRad);
        hoodAngleRad = arc[0];
        launchSpeedMps = arc[1];
        tof = arc[2];
      }
    } else {
      // Full motion-compensated path with convergence early-exit
      double lastDist = 0.0;
      for (int i = 0; i < kLeadIterations; i++) {
        aimTarget = KinematicsHelper.predictTargetPos(staticTarget, pivotVx, pivotVy, tof);
        double dist = turretPivotField.getDistance(aimTarget.toTranslation2d());

        if (i > 0 && Math.abs(dist - lastDist) < kConvergeThresholdDist) {
          leadConvergedEarly = true;
          break;
        }
        lastDist = dist;
        leadSolveIterations++;

        if (effectiveHubMode) {
          double scaledFunnelRadius =
              staticDistM > 1e-6
                  ? Constants.kHubFunnelRadiusMeters * dist / staticDistM
                  : Constants.kHubFunnelRadiusMeters;
          double funnelClearHeight =
              Constants.kHubFunnelHeightMeters + Constants.kHubFunnelClearanceMeters;
          lastDistM = dist;
          lastFunnelRadiusM = scaledFunnelRadius;
          lastFunnelClearHeightM = funnelClearHeight;
          double fallbackSpeedMps = (ShooterConstants.kAutoAimRPM / 60.0) * velConstant;
          double distFromLaunch = Math.max(0.01, dist - kLaunchOffsetMeters);

          var solved =
              KinematicsHelper.solveFunnelClearance(
                  distFromLaunch,
                  launchHeight,
                  aimTarget.getZ(),
                  scaledFunnelRadius,
                  funnelClearHeight,
                  fallbackSpeedMps);
          hoodAngleRad = solved.hoodAngleRad();
          launchSpeedMps = solved.launchSpeedMps();
          tof = solved.timeOfFlightSec();
          lastEffectiveDistFromLaunchM = solved.effectiveDistM();
          lastFunnelClearHeightM = solved.effectiveFunnelClearHeightM();
          lastFunnelConstraintMet = solved.funnelConstraintMet();
          if (lastFunnelConstraintMet && firstFunnelValidIteration < 0) {
            firstFunnelValidIteration = i;
          }
        } else {
          if (isPassCenter()) {
            double[] highArc =
                KinematicsHelper.solveForTargetHighArc(
                    dist, launchHeight, aimTarget.getZ(), maxHoodRad);
            hoodAngleRad = highArc[0];
            launchSpeedMps = highArc[1];
            tof = highArc[2];
          } else {
            double[] lowArc =
                KinematicsHelper.solveForTargetHighArc(
                    dist, launchHeight, aimTarget.getZ(), HoodConstants.kMinAngle.in(Radians));
            hoodAngleRad = lowArc[0];
            launchSpeedMps = lowArc[1];
            tof = lowArc[2];
          }
        }
      }
    }

    boolean warmstartTableUsed = effectiveHubMode && leadSolveIterations > 0;
    double warmstartLookupDistM =
        warmstartTableUsed ? Math.max(0.01, lastDistM - kLaunchOffsetMeters) : 0.0;
    ShotWarmstartTable.DistanceRegion warmstartRegion =
        warmstartTableUsed ? ShotWarmstartTable.distanceRegion(warmstartLookupDistM) : null;
    int iterationsUntilFunnelValid =
        firstFunnelValidIteration >= 0 ? firstFunnelValidIteration + 1 : 0;

    double rpm = (launchSpeedMps / velConstant) * 60.0;
    rpm = Math.max(ShooterConstants.kAutoAimRPMMin, Math.min(ShooterConstants.kAutoAimRPMMax, rpm));
    if (!effectiveHubMode && !isPassCenter()) rpm = Math.min(rpm, 6000);

    final double kObstacleZoneXMin = 4.472;
    final double kObstacleZoneXMax = 5.0;
    final double kTurretObstacleRadiusM = 0.33;
    double pivotX = turretPivotField.getX();
    double blueBandMin = kObstacleZoneXMin - kTurretObstacleRadiusM;
    double blueBandMax = kObstacleZoneXMax + kTurretObstacleRadiusM;
    double redBandMin = kFieldLengthMeters - kObstacleZoneXMax - kTurretObstacleRadiusM;
    double redBandMax = kFieldLengthMeters - kObstacleZoneXMin + kTurretObstacleRadiusM;
    boolean inObstacleZone =
        (pivotX >= blueBandMin && pivotX <= blueBandMax)
            || (pivotX >= redBandMin && pivotX <= redBandMax);
    if (inObstacleZone) {
      hoodAngleRad = HoodConstants.kMaxAngle.in(Radians);
    }

    shooter.setVelocity(RPM.of(rpm));

    if (RobotState.getInstance().isShooting()) hood.setAngle(Radians.of(hoodAngleRad));
    else hood.setAngle(HoodConstants.kMaxAngle);

    Translation2d aimPoint2d = aimTarget.toTranslation2d();
    if (effectiveHubMode && lastEffectiveDistFromLaunchM > 0) {
      Translation2d toHub = aimTarget.toTranslation2d().minus(turretPivotField);
      double norm = toHub.getNorm();
      if (norm >= 1e-6) {
        double effectivePivotToHub = lastEffectiveDistFromLaunchM + kLaunchOffsetMeters;
        aimPoint2d = turretPivotField.plus(toHub.times(effectivePivotToHub / norm));
      }
    }
    double desiredRaw =
        KinematicsHelper.getDesiredTurretAngleRadHalfTurn(robotPose, turretPivotField, aimPoint2d);
    double minAngleRad = TurretConstants.kAbsoluteMinAngle.in(Radians);
    double maxAngleRad = TurretConstants.kAbsoluteMaxAngle.in(Radians);

    double desiredTurretRad;
    if (!allowFullRange) {
      desiredTurretRad = MathUtil.clamp(desiredRaw, -Math.PI, Math.PI);
    } else {
      desiredTurretRad =
          KinematicsHelper.calculateAzimuthAngleRad(
              robotPose, turretPivotField, aimPoint2d, turret.getAngle().in(Radians));
    }
    desiredTurretRad = MathUtil.clamp(desiredTurretRad, minAngleRad, maxAngleRad);

    // Low-pass filter on turret angle to reduce jitter
    filteredAimAngle =
        kAimFilterAlpha * desiredTurretRad + (1.0 - kAimFilterAlpha) * filteredAimAngle;
    desiredTurretRad = filteredAimAngle;
    desiredTurretRad = MathUtil.clamp(desiredTurretRad, minAngleRad, maxAngleRad);

    double turretOmega =
        -headingRate; // feedforward tracks moving setpoint (same ω as pose rotation)

    turret.setAngleWithVelocity(Radians.of(desiredTurretRad), RadiansPerSecond.of(turretOmega));

    boolean turretClamped =
        (desiredTurretRad - minAngleRad < 1e-4) || (maxAngleRad - desiredTurretRad < 1e-4);

    boolean arcValid = (effectiveHubMode ? lastFunnelConstraintMet : true) && !turretClamped;
    state.setAutoAimArcValid(arcValid);

    // Critical outputs: always log
    Logger.recordOutput("AutoAim/ValidShot", arcValid);
    Logger.recordOutput("AutoAim/LeadSolveIterations", leadSolveIterations);
    Logger.recordOutput("AutoAim/LeadConvergedEarly", leadConvergedEarly);
    Logger.recordOutput(
        "AutoAim/FirstFunnelValidIteration", effectiveHubMode ? firstFunnelValidIteration : -1);
    Logger.recordOutput(
        "AutoAim/IterationsUntilFunnelValid", effectiveHubMode ? iterationsUntilFunnelValid : 0);
    Logger.recordOutput("AutoAim/WarmstartTableUsed", warmstartTableUsed);
    Logger.recordOutput(
        "AutoAim/WarmstartMapRegion", warmstartRegion != null ? warmstartRegion.name() : "N_A");
    Logger.recordOutput("AutoAim/WarmstartLookupDistM", warmstartLookupDistM);
    Logger.recordOutput("AutoAim/DistanceM", turretPivotField.getDistance(aimPoint2d));
    Logger.recordOutput("AutoAim/TOF", tof);
    Logger.recordOutput("AutoAim/HoodAngleCommandedDeg", Math.toDegrees(hoodAngleRad));
    Logger.recordOutput("AutoAim/TurretDesiredDeg", Math.toDegrees(desiredTurretRad));
    Logger.recordOutput(
        "AutoAim/TurretErrorDeg", Math.toDegrees(desiredTurretRad - turret.getAngle().in(Radians)));
    Logger.recordOutput("AutoAim/TurretClamped", turretClamped);

    // Verbose diagnostics: log every N cycles to reduce overhead
    boolean logThisCycle = (logCounter++ % kLogInterval == 0);
    if (logThisCycle) {
      Logger.recordOutput(
          "AutoAim/FunnelConstraintMet", effectiveHubMode ? lastFunnelConstraintMet : true);
      Logger.recordOutput("AutoAim/Inputs/RobotPose", robotPose);
      Logger.recordOutput("AutoAim/Inputs/HubModeRequested", hubMode);
      Logger.recordOutput("AutoAim/Inputs/HubModeEffective", effectiveHubMode);
      Logger.recordOutput("AutoAim/Inputs/PastAllianceLine", pastAllianceLine);
      Logger.recordOutput(
          "AutoAim/Inputs/PassMode",
          isPassCenter() ? PassMode.CENTER.name() : PassMode.SIDES.name());
      Logger.recordOutput("AutoAim/Inputs/StaticTarget", staticTarget);
      Logger.recordOutput("AutoAim/Inputs/TurretPivotField", turretPivotField);
      Logger.recordOutput("AutoAim/Inputs/LaunchSpeedMps", launchSpeedMps);
      Logger.recordOutput("AutoAim/Inputs/RPM", rpm);
      Logger.recordOutput("AutoAim/Inputs/StaticDistM", staticDistM);
      Logger.recordOutput("AutoAim/Inputs/VelConstant", velConstant);
      Logger.recordOutput("AutoAim/Inputs/LaunchHeightM", launchHeight);
      Logger.recordOutput("AutoAim/Inputs/RobotVx", robotVx);
      Logger.recordOutput("AutoAim/Inputs/RobotVy", robotVy);
      Logger.recordOutput("AutoAim/Inputs/RobotOmega", robotOmega);
      Logger.recordOutput("AutoAim/Inputs/HeadingRateRadPerSec", headingRate);
      Logger.recordOutput("AutoAim/Inputs/PivotVx", pivotVx);
      Logger.recordOutput("AutoAim/Inputs/PivotVy", pivotVy);
      Logger.recordOutput("AutoAim/Inputs/PoseAgeSec", poseAgeSec);
      Logger.recordOutput("AutoAim/Inputs/Stationary", isStationary);
      Logger.recordOutput("AutoAim/AimTarget", aimTarget);
      Logger.recordOutput("AutoAim/AimPoint", aimPoint2d);
      Logger.recordOutput(
          "AutoAim/StaticDistanceM", turretPivotField.getDistance(staticTarget.toTranslation2d()));
      Logger.recordOutput(
          "AutoAim/HoodAngleActualDeg", Math.toDegrees(hood.getAngle().in(Radians)));
      Logger.recordOutput("AutoAim/TurretActualDeg", Math.toDegrees(turret.getAngle().in(Radians)));
      Logger.recordOutput("AutoAim/TurretOmega", turretOmega);

      if (effectiveHubMode && lastDistM > 1e-6) {
        Translation2d toHub = aimTarget.toTranslation2d().minus(turretPivotField);
        double norm = toHub.getNorm();
        double effectivePivotToHub =
            lastEffectiveDistFromLaunchM > 0
                ? lastEffectiveDistFromLaunchM + kLaunchOffsetMeters
                : lastDistM;
        double pivotToFunnel = Math.max(0.0, effectivePivotToHub - lastFunnelRadiusM);
        Translation2d funnelXY =
            norm >= 1e-6
                ? turretPivotField.plus(toHub.div(norm).times(pivotToFunnel))
                : turretPivotField;
        Pose3d funnelClearancePose =
            new Pose3d(funnelXY.getX(), funnelXY.getY(), lastFunnelClearHeightM, new Rotation3d());
        Pose3d hubCenterPose =
            new Pose3d(aimPoint2d.getX(), aimPoint2d.getY(), aimTarget.getZ(), new Rotation3d());
        Logger.recordOutput("AutoAim/FunnelClearancePose", funnelClearancePose);
        Logger.recordOutput("AutoAim/HubCenterPose", hubCenterPose);
        Logger.recordOutput("AutoAim/FunnelClearanceZM", lastFunnelClearHeightM);
        Logger.recordOutput("AutoAim/HubCenterZM", aimTarget.getZ());
      }

      Translation3d[] traj =
          buildTrajectory(
              robotPose, hoodAngleRad, desiredTurretRad, pivotVx, pivotVy, launchSpeedMps);
      Logger.recordOutput("AutoAim/Trajectory", traj);
      if (traj.length > 0) {
        Translation3d end = traj[traj.length - 1];
        Logger.recordOutput("AutoAim/TrajectoryEndpoint", end);
        Logger.recordOutput("AutoAim/TrajectoryEndXErrorM", end.getX() - staticTarget.getX());
        Logger.recordOutput("AutoAim/TrajectoryEndYErrorM", end.getY() - staticTarget.getY());
        Logger.recordOutput("AutoAim/TrajectoryEndZM", end.getZ());
      }
    }
  }

  /** Command that runs auto-aim every cycle. allowFullRange: true when shooting. */
  public static Command autoAim(
      Turret turret,
      Shooter shooter,
      Hood hood,
      BooleanSupplier hubMode,
      BooleanSupplier allowFullRange) {
    return run(
        () ->
            updateAutoAim(
                turret,
                shooter,
                hood,
                hubMode.getAsBoolean(),
                allowFullRange != null && allowFullRange.getAsBoolean()),
        turret,
        shooter,
        hood);
  }

  private static final LoggedNetworkNumber launchVelConstant =
      new LoggedNetworkNumber(
          "AutoAim/LaunchVelConstant", ShooterConstants.kFuelLaunchVelMetersPerSecPerRotPerSec);

  public enum PassMode {
    CENTER,
    SIDES
  }

  private static final LoggedDashboardChooser<PassMode> passModeChooser;

  static {
    passModeChooser = new LoggedDashboardChooser<>("Pass Mode");
    passModeChooser.addDefaultOption("Side Pass", PassMode.SIDES);
    passModeChooser.addOption("Center Pass", PassMode.CENTER);
  }

  private static final Translation2d kPassCenterBlue = new Translation2d(2.4, 4.0);

  private static boolean isPassCenter() {
    PassMode mode = passModeChooser.get();
    return mode == PassMode.CENTER;
  }

  public static double getLaunchVelConstant() {
    return launchVelConstant.get();
  }

  public static boolean isPastAllianceLine(Pose2d robotPose, boolean isRedAlliance) {
    return isRedAlliance
        ? robotPose.getX() < (kFieldLengthMeters - kAllianceLineXBlue)
        : robotPose.getX() > kAllianceLineXBlue;
  }

  public static boolean useHubMode(
      boolean requestedHubMode, Pose2d robotPose, boolean isRedAlliance) {
    return requestedHubMode && !isPastAllianceLine(robotPose, isRedAlliance);
  }

  private static final double kLaunchVelConstantStep = 0.01;

  public static void decreaseLaunchVelConstant() {
    double v = Math.max(0.0, launchVelConstant.get() - kLaunchVelConstantStep);
    launchVelConstant.set(v);
  }

  public static void increaseLaunchVelConstant() {
    launchVelConstant.set(launchVelConstant.get() + kLaunchVelConstantStep);
  }

  private static Translation3d resolveTarget(Pose2d robotPose, RobotState state, boolean hubMode) {
    if (hubMode) {
      Translation2d hub =
          allianceFlip(Constants.kHubPoseBlue.getTranslation(), state.isRedAlliance());
      return new Translation3d(hub.getX(), hub.getY(), Constants.kHubTargetHeightMeters);
    }
    if (isPassCenter()) {
      Translation2d pass = allianceFlip(kPassCenterBlue, state.isRedAlliance());
      return new Translation3d(pass.getX(), pass.getY(), 0.0);
    }
    Translation2d passingBlue =
        robotPose.getY() > Constants.kPassingYThresholdMeters
            ? Constants.kPassingPoseHighBlue.getTranslation()
            : Constants.kPassingPoseLowBlue.getTranslation();
    Translation2d pass = allianceFlip(passingBlue, state.isRedAlliance());
    return new Translation3d(pass.getX(), pass.getY(), 0.0);
  }

  private static Translation2d allianceFlip(Translation2d blue, boolean isRed) {
    return isRed ? new Translation2d(kFieldLengthMeters - blue.getX(), blue.getY()) : blue;
  }

  /** Build trajectory for visualization from current state (SIM/REPLAY). */
  public static Translation3d[] buildTrajectoryFromState(
      Pose2d robotPose,
      double hoodRad,
      double turretRad,
      double pivotVx,
      double pivotVy,
      double launchSpeedMps) {
    return buildTrajectory(robotPose, hoodRad, turretRad, pivotVx, pivotVy, launchSpeedMps);
  }

  private static Translation3d[] buildTrajectory(
      Pose2d robotPose,
      double hoodRad,
      double turretRad,
      double pivotVx,
      double pivotVy,
      double launchSpeedMps) {
    double launchSpeed = launchSpeedMps;

    double ch = Math.cos(hoodRad), sh = Math.sin(hoodRad);
    double ct = Math.cos(turretRad), st = Math.sin(turretRad);

    double tx = Constants.turretBasePose.getX();
    double ty = Constants.turretBasePose.getY();
    double hz = Constants.hoodBasePose.getZ();
    double lx = tx + kLaunchOffsetMeters * ch * ct;
    double ly = ty + kLaunchOffsetMeters * ch * st;
    double lz = hz + kLaunchOffsetMeters * sh;

    double rCos = robotPose.getRotation().getCos();
    double rSin = robotPose.getRotation().getSin();
    double x0 = robotPose.getX() + lx * rCos - ly * rSin;
    double y0 = robotPose.getY() + lx * rSin + ly * rCos;

    double fieldYaw = robotPose.getRotation().getRadians() + turretRad;
    double hVel = launchSpeed * ch;
    double vx = hVel * Math.cos(fieldYaw) + pivotVx;
    double vy = hVel * Math.sin(fieldYaw) + pivotVy;
    double vz = launchSpeed * sh;

    Translation3d[] pts = new Translation3d[kTrajectoryMaxPts];
    int n = 0;
    for (double t = 0; n < kTrajectoryMaxPts; t += kTrajectoryDt) {
      double z = lz + vz * t - 0.5 * kGravity * t * t;
      pts[n++] = new Translation3d(x0 + vx * t, y0 + vy * t, z);
      if (t > 0 && z <= 0) break;
    }

    Translation3d[] result = new Translation3d[n];
    System.arraycopy(pts, 0, result, 0, n);
    return result;
  }
}
