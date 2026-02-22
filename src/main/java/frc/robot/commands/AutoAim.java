package frc.robot.commands;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.wpilibj2.command.Commands.run;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
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
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

/** Auto-aim: hub shots (funnel solver) or pass shots (drag solver). */
public final class AutoAim {

  private static final double kFieldLengthMeters = 16.54;
  private static final double kLaunchOffsetMeters = 0.28;
  private static final double kGravity = 9.81;
  private static final double kTrajectoryDt = 0.02;
  private static final int kTrajectoryMaxPts = 128;
  private static final int kLeadIterations = 3;
  private static final double kTwoPi = 2.0 * Math.PI;

  /** Previous commanded turret angle (rad) for wrap choice; avoids double-wrap when shooting. */
  private static double previousCommandedTurretRad = Double.NaN;

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
    if (latestPose == null) return;

    Pose2d robotPose = latestPose.getValue();
    Translation3d staticTarget = resolveTarget(robotPose, state, hubMode);

    Translation2d turretPivotFieldEarly = KinematicsHelper.getTurretPivotTranslation(robotPose);
    double staticDistM = turretPivotFieldEarly.getDistance(staticTarget.toTranslation2d());

    double velConstant = getLaunchVelConstant();
    double launchHeight = Constants.hoodBasePose.getZ();

    Translation2d turretPivotField = turretPivotFieldEarly;

    var fieldSpeeds = state.getLatestMeasuredFieldRelativeChassisSpeeds();
    double robotVx = fieldSpeeds != null ? fieldSpeeds.vxMetersPerSecond : 0.0;
    double robotVy = fieldSpeeds != null ? fieldSpeeds.vyMetersPerSecond : 0.0;
    double robotOmega = fieldSpeeds != null ? fieldSpeeds.omegaRadiansPerSecond : 0.0;

    Translation2d pivotOffset = turretPivotField.minus(robotPose.getTranslation());
    double pivotVx = robotVx - robotOmega * pivotOffset.getY();
    double pivotVy = robotVy + robotOmega * pivotOffset.getX();

    double minHoodRad = HoodConstants.kMinAngle.in(Radians);
    double maxHoodRad = HoodConstants.kMaxAngle.in(Radians);

    // Iterate to converge on motion-compensated aim
    Translation3d aimTarget = staticTarget;
    double hoodAngleRad = 0.0;
    double tof = 0.0;
    double launchSpeedMps = 5.0;
    double lastDistM = 0.0;
    double lastFunnelRadiusM = 0.0;
    double lastFunnelClearHeightM = 0.0;
    double lastEffectiveDistFromLaunchM = 0.0;
    boolean lastFunnelConstraintMet = true;

    for (int i = 0; i < kLeadIterations; i++) {
      aimTarget = KinematicsHelper.predictTargetPos(staticTarget, pivotVx, pivotVy, tof);
      double dist = turretPivotField.getDistance(aimTarget.toTranslation2d());

      if (hubMode) {
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
      } else {
        launchSpeedMps = (getPassRPM() / 60.0) * velConstant;
        var solved =
            KinematicsHelper.solveForTarget(
                dist, launchHeight, aimTarget.getZ(), launchSpeedMps, minHoodRad, maxHoodRad);
        hoodAngleRad = solved.hoodAngleRad();
        tof = solved.timeOfFlightSec();
      }
    }

    double rpm = (launchSpeedMps / velConstant) * 60.0;
    rpm = Math.max(ShooterConstants.kAutoAimRPMMin, Math.min(ShooterConstants.kAutoAimRPMMax, rpm));

    shooter.setRPM(rpm);

    hood.setAngle(Radians.of(hoodAngleRad));

    Translation2d aimPoint2d = aimTarget.toTranslation2d();
    if (hubMode && lastEffectiveDistFromLaunchM > 0) {
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
      double ref =
          Double.isNaN(previousCommandedTurretRad)
              ? turret.getAngle().in(Radians)
              : previousCommandedTurretRad;
      desiredTurretRad = desiredRaw + kTwoPi * Math.round((ref - desiredRaw) / kTwoPi);
      if (!Double.isNaN(previousCommandedTurretRad)
          && Math.abs(desiredTurretRad - previousCommandedTurretRad) > Math.PI) {
        double alt =
            desiredTurretRad - Math.signum(desiredTurretRad - previousCommandedTurretRad) * kTwoPi;
        if (alt >= minAngleRad && alt <= maxAngleRad) desiredTurretRad = alt;
      }
    }
    desiredTurretRad = MathUtil.clamp(desiredTurretRad, minAngleRad, maxAngleRad);
    previousCommandedTurretRad = desiredTurretRad;

    double turretOmega = -robotOmega; // counter-rotate to hold field heading

    turret.setAngleWithVelocity(Radians.of(desiredTurretRad), RadiansPerSecond.of(turretOmega));

    double minAngle = TurretConstants.kAbsoluteMinAngle.in(Radians);
    double maxAngle = TurretConstants.kAbsoluteMaxAngle.in(Radians);
    boolean turretClamped =
        (desiredTurretRad - minAngle < 1e-4) || (maxAngle - desiredTurretRad < 1e-4);

    Logger.recordOutput("AutoAim/ValidShot", true);
    Logger.recordOutput("AutoAim/FunnelConstraintMet", hubMode ? lastFunnelConstraintMet : true);
    Logger.recordOutput("AutoAim/Inputs/RobotPose", robotPose);
    Logger.recordOutput("AutoAim/Inputs/HubMode", hubMode);
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
    Logger.recordOutput("AutoAim/Inputs/PivotVx", pivotVx);
    Logger.recordOutput("AutoAim/Inputs/PivotVy", pivotVy);
    Logger.recordOutput("AutoAim/AimTarget", aimTarget);
    Logger.recordOutput("AutoAim/AimPoint", aimPoint2d);

    if (hubMode && lastDistM > 1e-6) {
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

    Logger.recordOutput("AutoAim/DistanceM", turretPivotField.getDistance(aimPoint2d));
    Logger.recordOutput(
        "AutoAim/StaticDistanceM", turretPivotField.getDistance(staticTarget.toTranslation2d()));
    Logger.recordOutput("AutoAim/TOF", tof);
    Logger.recordOutput("AutoAim/HoodAngleCommandedDeg", Math.toDegrees(hoodAngleRad));
    Logger.recordOutput("AutoAim/HoodAngleActualDeg", Math.toDegrees(hood.getAngle().in(Radians)));
    Logger.recordOutput("AutoAim/TurretDesiredDeg", Math.toDegrees(desiredTurretRad));
    Logger.recordOutput("AutoAim/TurretActualDeg", Math.toDegrees(turret.getAngle().in(Radians)));
    Logger.recordOutput(
        "AutoAim/TurretErrorDeg", Math.toDegrees(desiredTurretRad - turret.getAngle().in(Radians)));
    Logger.recordOutput("AutoAim/TurretClamped", turretClamped);
    Logger.recordOutput("AutoAim/TurretOmega", turretOmega);

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

  /** Command that runs auto-aim every cycle. allowFullRange: true when shooting (use full 720°). */
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

  private static final LoggedNetworkNumber passRPMOverride =
      new LoggedNetworkNumber("AutoAim/PassRPMOverride", -1.0);

  private static final LoggedNetworkNumber launchVelConstant =
      new LoggedNetworkNumber(
          "AutoAim/LaunchVelConstant", ShooterConstants.kFuelLaunchVelMetersPerSecPerRotPerSec);

  private static double getPassRPM() {
    double override = passRPMOverride.get();
    if (override >= ShooterConstants.kAutoAimRPMMin) {
      return Math.min(override, ShooterConstants.kAutoAimRPMMax);
    }
    return ShooterConstants.kAutoAimRPM;
  }

  public static double getLaunchVelConstant() {
    return launchVelConstant.get();
  }

  private static Translation3d resolveTarget(Pose2d robotPose, RobotState state, boolean hubMode) {
    if (hubMode) {
      Translation2d hub =
          allianceFlip(Constants.kHubPoseBlue.getTranslation(), state.isRedAlliance());
      return new Translation3d(hub.getX(), hub.getY(), Constants.kHubTargetHeightMeters);
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
