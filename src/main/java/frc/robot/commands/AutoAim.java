package frc.robot.commands;

import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.wpilibj2.command.Commands.run;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotState;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.hood.HoodConstants;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterConstants;
import frc.robot.subsystems.turret.Turret;
import frc.robot.util.KinematicsHelper;
import frc.robot.util.ShotCalculator;
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

/** Auto-aim for stationary and moving shots using one shared calculator. */
public final class AutoAim {

  private static final double kFieldLengthMeters = 16.54;
  private static final double kAllianceLineXBlue = 4.5;
  private static final double kLaunchOffsetMeters = ShotCalculator.LAUNCH_OFFSET_M;
  private static final double kGravity = 9.81;
  private static final double kTrajectoryDt = 0.02;
  private static final int kTrajectoryMaxPts = 128;

  private AutoAim() {}

  /** One cycle: set shooter RPM, hood angle, turret heading. Default: turret in [-180°, 180°]. */
  public static void updateAutoAim(Turret turret, Shooter shooter, Hood hood, boolean hubMode) {
    updateAutoAim(turret, shooter, hood, hubMode, false);
  }

  private static final ShotCalculator calculator = new ShotCalculator();
  private static final LoggedNetworkNumber releaseDelay =
      new LoggedNetworkNumber("AutoAim/ReleaseDelaySec", 0.0);
  private static final LoggedNetworkBoolean measuredCalibration =
      new LoggedNetworkBoolean("AutoAim/UseMeasuredCalibration", false);
  private static final LoggedNetworkBoolean detailedTrajectory =
      new LoggedNetworkBoolean("AutoAim/DetailedTrajectory", false);
  private static boolean activeCalibration;
  private static boolean wasEnabled;
  private static boolean updatedThisCycle;
  /** Called before CommandScheduler.run(); calibration changes latch only while disabled. */
  public static void beginCycle() {
    boolean enabled = DriverStation.isEnabled();
    if (!enabled && activeCalibration != measuredCalibration.get()) {
      calculator.resetTracking();
      activeCalibration = measuredCalibration.get();
    }
    if (enabled != wasEnabled || !updatedThisCycle) calculator.resetTracking();
    wasEnabled = enabled;
    updatedThisCycle = false;
    calculator.beginCycle();
    Logger.recordOutput("AutoAim/ActiveMode", "MOVING");
    Logger.recordOutput("AutoAim/MeasuredCalibrationEnabled", activeCalibration);
  }

  public static boolean shouldLogTrajectory() {
    return Constants.currentMode != Constants.Mode.REAL || detailedTrajectory.get();
  }

  /** Uses the configured physical travel range while shooting, otherwise [-180, 180] degrees. */
  public static void updateAutoAim(
      Turret turret, Shooter shooter, Hood hood, boolean hubMode, boolean allowFullRange) {
    if (updatedThisCycle) return;
    updatedThisCycle = true;
    long started = System.nanoTime();
    RobotState state = RobotState.getInstance();
    var pose = state.getLatestFieldToRobot();
    if (pose == null) {
      state.setAutoAimArcValid(false);
      return;
    }
    var speeds = state.getLatestRobotRelativeChassisSpeed();
    double now = Timer.getFPGATimestamp();
    double delay = releaseDelay.get();
    Pose2d releasePose =
        ShotCalculator.predictReleasePose(
            pose.getValue(),
            speeds.vxMetersPerSecond,
            speeds.vyMetersPerSecond,
            state.getHeadingRateRadPerSec(),
            MathUtil.clamp(now - pose.getKey(), 0.0, ShotCalculator.MAX_POSE_AGE_SEC)
                + (Double.isFinite(delay)
                    ? MathUtil.clamp(delay, 0.0, ShotCalculator.MAX_RELEASE_DELAY_SEC)
                    : 0.0));
    boolean effectiveHub = useHubMode(hubMode, releasePose, state.isRedAlliance());
    long solveStarted = System.nanoTime();
    var shot =
        calculator.calculate(
            new ShotCalculator.Input(
                now,
                pose.getKey(),
                pose.getValue(),
                speeds.vxMetersPerSecond,
                speeds.vyMetersPerSecond,
                state.getHeadingRateRadPerSec(),
                turret.getAngle().in(Radians),
                resolveTarget(releasePose, state, effectiveHub),
                effectiveHub,
                isPassCenter(),
                allowFullRange,
                getLaunchVelConstant(),
                delay,
                true,
                activeCalibration,
                state.getPoseResetSequence(),
                getTurretOmegaAlpha()));
    Logger.recordOutput("ShotCalculator/SolveTimeMs", (System.nanoTime() - solveStarted) / 1e6);
    logSolution(shot, turret, shooter, hood, now - pose.getKey());
    shooter.setVelocity(RPM.of(state.isShooting() ? shot.wheelRpm() : 0.0));
    double hoodRad = shot.hoodAngleRad();
    // Preserve the obstacle override even when the calculator is holding a prior finite shot.
    if (ShotCalculator.inObstacleZone(
        KinematicsHelper.getTurretPivotTranslation(pose.getValue()).getX()))
      hoodRad = HoodConstants.kMaxAngle.in(Radians);
    hood.setAngle(state.isShooting() ? Radians.of(hoodRad) : HoodConstants.kMaxAngle);
    turret.setAngleWithVelocity(
        Radians.of(shot.turretAngleRad()), RadiansPerSecond.of(shot.turretVelocityRadPerSec()));
    state.setAutoAimArcValid(shot.valid());
    Logger.recordOutput("AutoAim/ValidShot", shot.valid());
    Logger.recordOutput("AutoAim/TOF", shot.timeOfFlightSec());
    Logger.recordOutput("AutoAim/DistanceM", shot.distanceM());
    Logger.recordOutput("AutoAim/TurretDesiredDeg", Math.toDegrees(shot.turretAngleRad()));
    Logger.recordOutput("AutoAim/HoodAngleCommandedDeg", Math.toDegrees(hoodRad));
    Logger.recordOutput("AutoAim/UpdateTimeMs", (System.nanoTime() - started) / 1e6);
  }

  private static void logSolution(
      ShotCalculator.Solution shot, Turret turret, Shooter shooter, Hood hood, double age) {
    double turretError = shot.turretAngleRad() - turret.getAngle().in(Radians);
    double hoodError = shot.hoodAngleRad() - hood.getAngle().in(Radians);
    double rpmError = shot.wheelRpm() - shooter.getRPM();
    Logger.recordOutput("ShotCalculator/Valid", shot.valid());
    Logger.recordOutput("ShotCalculator/Reason", shot.reason());
    Logger.recordOutput("ShotCalculator/Converged", shot.converged());
    Logger.recordOutput("ShotCalculator/Iterations", shot.iterations());
    Logger.recordOutput("ShotCalculator/PoseAgeSec", age);
    Logger.recordOutput("ShotCalculator/ReleasePose", shot.releasePose());
    Logger.recordOutput("ShotCalculator/VirtualTarget", shot.virtualTarget());
    Logger.recordOutput("ShotCalculator/PivotVelocity", shot.pivotVelocity());
    Logger.recordOutput("ShotCalculator/TOF", shot.timeOfFlightSec());
    Logger.recordOutput("ShotCalculator/WheelRpm", shot.wheelRpm());
    Logger.recordOutput("ShotCalculator/HoodAngleDeg", Math.toDegrees(shot.hoodAngleRad()));
    Logger.recordOutput("ShotCalculator/TurretAngleDeg", Math.toDegrees(shot.turretAngleRad()));
    Logger.recordOutput("ShotCalculator/DistanceM", shot.distanceM());
    Logger.recordOutput("ShotCalculator/MeasuredCalibration", shot.measuredCalibration());
    Logger.recordOutput("ShotCalculator/TurretVelocityRadPerSec", shot.turretVelocityRadPerSec());
    Logger.recordOutput("ShotCalculator/TurretErrorDeg", Math.toDegrees(turretError));
    Logger.recordOutput("ShotCalculator/HoodErrorDeg", Math.toDegrees(hoodError));
    Logger.recordOutput("ShotCalculator/WheelRpmError", rpmError);
    boolean turretReady = Math.abs(turretError) <= Math.toRadians(2.0);
    boolean hoodReady = Math.abs(hoodError) <= Math.toRadians(2.0);
    boolean rpmReady =
        shot.wheelRpm() > 100 && Math.abs(rpmError) <= ShooterConstants.kRpmToleranceForReady;
    Logger.recordOutput("ShotCalculator/Readiness/Turret", turretReady);
    Logger.recordOutput("ShotCalculator/Readiness/Hood", hoodReady);
    Logger.recordOutput("ShotCalculator/Readiness/Shooter", rpmReady);
    Logger.recordOutput(
        "ShotCalculator/Readiness/All",
        RobotState.getInstance().isShooting()
            && shot.valid()
            && turretReady
            && hoodReady
            && rpmReady);
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
  private static final LoggedNetworkNumber turretOmegaAlpha =
      new LoggedNetworkNumber("AutoAim/TurretOmegaAlpha", 1.0);

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

  public static double getTurretOmegaAlpha() {
    return turretOmegaAlpha.get();
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
