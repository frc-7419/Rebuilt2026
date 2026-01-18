package frc.robot.simulation;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotState;
import frc.robot.subsystems.shooter.ShooterConstants;
import frc.robot.subsystems.turret.TurretConstants;
import java.util.ArrayList;
import java.util.List;
import java.util.Random;
import java.util.concurrent.ConcurrentHashMap;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class VisualizeFuelShot extends Command {

  public static Command visualizeFuelShot() {
    Supplier<MotionParams> supplier =
        () -> {
          RobotState state = RobotState.getInstance();

          MotionParams params = new MotionParams();

          var latestRobotPose = state.getLatestFieldToRobot();
          var latestTurretRotation = state.getLatestRobotToTurret();

          Logger.recordOutput("FuelVisualizer/Inputs/RobotPoseAvailable", latestRobotPose != null);
          Logger.recordOutput(
              "FuelVisualizer/Inputs/TurretRotationAvailable", latestTurretRotation != null);

          if (latestRobotPose == null || latestTurretRotation == null) {
            Logger.recordOutput("FuelVisualizer/Error", "Missing required state data");
            return params;
          }

          Pose2d robotPose = latestRobotPose.getValue();
          Rotation2d turretRotation = latestTurretRotation.getValue();
          turretRotation = robotPose.getRotation().rotateBy(turretRotation);

          double hoodAngleRad =
              state.getCurrentHoodPosition().in(Radians) - ShooterConstants.kHoodZeroed.in(Radians);

          double hoodAngleUncertaintyDeg = 0.25 + random.nextDouble() * (1.0 - 0.25);
          double hoodAngleUncertaintySign = random.nextBoolean() ? 1.0 : -1.0;
          hoodAngleRad += Math.toRadians(hoodAngleUncertaintyDeg * hoodAngleUncertaintySign);

          double wheelRotPerSec = state.getCurrentShooterVelocity().in(RotationsPerSecond);

          Logger.recordOutput("FuelVisualizer/Inputs/RobotPoseX", robotPose.getX());
          Logger.recordOutput("FuelVisualizer/Inputs/RobotPoseY", robotPose.getY());
          Logger.recordOutput(
              "FuelVisualizer/Inputs/RobotPoseRotationDeg", robotPose.getRotation().getDegrees());
          Logger.recordOutput(
              "FuelVisualizer/Inputs/TurretRotationDeg", turretRotation.getDegrees());
          Logger.recordOutput("FuelVisualizer/Inputs/HoodAngleRad", hoodAngleRad);
          Logger.recordOutput("FuelVisualizer/Inputs/HoodAngleDeg", Math.toDegrees(hoodAngleRad));
          Logger.recordOutput("FuelVisualizer/Inputs/WheelRotPerSec", wheelRotPerSec);

          // Calculate fuel exit velocity from wheel rotation rate
          double fuelExitMps =
              wheelRotPerSec * ShooterConstants.kFuelLaunchVelMetersPerSecPerRotPerSec;

          double speedUncertaintyPercent = random.nextDouble() * 5.0;
          double speedUncertaintySign = random.nextBoolean() ? 1.0 : -1.0;
          fuelExitMps *= (1.0 + speedUncertaintySign * speedUncertaintyPercent / 100.0);

          Logger.recordOutput("FuelVisualizer/Inputs/FuelExitMps", fuelExitMps);

          // Calculate wheel surface speed for spin calculation
          double wheelRadiusM = ShooterConstants.kShooterWheelRadius.in(Meters);
          double wheelSurfaceMps = wheelRotPerSec * 2.0 * Math.PI * wheelRadiusM;

          Logger.recordOutput("FuelVisualizer/Inputs/WheelSurfaceMps", wheelSurfaceMps);

          Translation3d velocity =
              new Translation3d(
                      fuelExitMps * Math.cos(hoodAngleRad),
                      0.0,
                      fuelExitMps * Math.sin(hoodAngleRad))
                  .rotateBy(new Rotation3d(0.0, 0.0, turretRotation.getRadians()));

          var chassisSpeeds = state.getLatestMeasuredFieldRelativeChassisSpeeds();
          var turretAngularVelocity = state.getLatestTurretAngularVelocity();
          double turretOmegaRadPerSec = turretAngularVelocity.in(RadiansPerSecond);
          double robotOmegaRadPerSec = chassisSpeeds.omegaRadiansPerSecond;

          Logger.recordOutput(
              "FuelVisualizer/Inputs/ChassisSpeedX", chassisSpeeds.vxMetersPerSecond);
          Logger.recordOutput(
              "FuelVisualizer/Inputs/ChassisSpeedY", chassisSpeeds.vyMetersPerSecond);
          Logger.recordOutput(
              "FuelVisualizer/Inputs/RobotAngularVelocityRadPerSec", robotOmegaRadPerSec);
          Logger.recordOutput(
              "FuelVisualizer/Inputs/TurretAngularVelocityRadPerSec", turretOmegaRadPerSec);

          double turretToShooterX =
              ShooterConstants.kRobotToShooterRelease.getX()
                  - TurretConstants.kTurretOffset.getTranslation().getX();
          double turretToShooterY =
              ShooterConstants.kRobotToShooterRelease.getY()
                  - TurretConstants.kTurretOffset.getTranslation().getY();
          double turretToShooterDistance = Math.hypot(turretToShooterX, turretToShooterY);

          double tangentialSpeed = turretOmegaRadPerSec * turretToShooterDistance;
          double tangentialXRobot = -turretToShooterY;
          double tangentialYRobot = turretToShooterX;

          if (turretToShooterDistance > 1e-6) {
            tangentialXRobot = (tangentialXRobot / turretToShooterDistance) * tangentialSpeed;
            tangentialYRobot = (tangentialYRobot / turretToShooterDistance) * tangentialSpeed;
          } else {
            tangentialXRobot = 0.0;
            tangentialYRobot = 0.0;
          }

          Translation3d tangentialVelocity =
              new Translation3d(tangentialXRobot, tangentialYRobot, 0.0)
                  .rotateBy(new Rotation3d(0.0, 0.0, turretRotation.getRadians()));

          Logger.recordOutput("FuelVisualizer/Inputs/TurretTangentialSpeed", tangentialSpeed);
          Logger.recordOutput(
              "FuelVisualizer/Inputs/TurretTangentialVelocityX", tangentialVelocity.getX());
          Logger.recordOutput(
              "FuelVisualizer/Inputs/TurretTangentialVelocityY", tangentialVelocity.getY());

          double robotToShooterX = ShooterConstants.kRobotToShooterRelease.getX();
          double robotToShooterY = ShooterConstants.kRobotToShooterRelease.getY();
          double robotToShooterDistance = Math.hypot(robotToShooterX, robotToShooterY);

          double robotTangentialSpeed = robotOmegaRadPerSec * robotToShooterDistance;
          double robotTangentialXRobot = -robotToShooterY;
          double robotTangentialYRobot = robotToShooterX;

          // Normalize and scale by tangential speed
          if (robotToShooterDistance > 1e-6) {
            robotTangentialXRobot =
                (robotTangentialXRobot / robotToShooterDistance) * robotTangentialSpeed;
            robotTangentialYRobot =
                (robotTangentialYRobot / robotToShooterDistance) * robotTangentialSpeed;
          } else {
            robotTangentialXRobot = 0.0;
            robotTangentialYRobot = 0.0;
          }

          Translation3d robotTangentialVelocity =
              new Translation3d(robotTangentialXRobot, robotTangentialYRobot, 0.0)
                  .rotateBy(new Rotation3d(0.0, 0.0, robotPose.getRotation().getRadians()));

          Logger.recordOutput("FuelVisualizer/Inputs/RobotTangentialSpeed", robotTangentialSpeed);
          Logger.recordOutput(
              "FuelVisualizer/Inputs/RobotTangentialVelocityX", robotTangentialVelocity.getX());
          Logger.recordOutput(
              "FuelVisualizer/Inputs/RobotTangentialVelocityY", robotTangentialVelocity.getY());

          velocity =
              velocity
                  .plus(
                      new Translation3d(
                          chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond, 0.0))
                  .plus(robotTangentialVelocity)
                  .plus(tangentialVelocity);

          params.startPose =
              new Pose3d(
                  robotPose.getX() + ShooterConstants.kRobotToShooterRelease.getX(),
                  robotPose.getY() + ShooterConstants.kRobotToShooterRelease.getY(),
                  ShooterConstants.kRobotToShooterRelease.getZ(),
                  new Rotation3d(0, -hoodAngleRad, turretRotation.getRadians()));

          params.velocity = velocity;

          double spinTransfer = ShooterConstants.kSpinTransfer;
          double omegaBallRadPerSec =
              ((wheelSurfaceMps - fuelExitMps) / kFuelRadiusMeters) * spinTransfer;

          double spinUncertaintyPercent = 5.0 + random.nextDouble() * (10.0 - 3.0);
          double spinUncertaintySign = random.nextBoolean() ? 1.0 : -1.0;
          omegaBallRadPerSec *= (1.0 + spinUncertaintySign * spinUncertaintyPercent / 100.0);

          omegaBallRadPerSec = clamp(omegaBallRadPerSec, -600.0, 600.0);

          Logger.recordOutput("FuelVisualizer/Inputs/WheelSurfaceMps", wheelSurfaceMps);
          Logger.recordOutput("FuelVisualizer/Inputs/OmegaBallRadPerSec", omegaBallRadPerSec);

          Translation3d omegaField =
              new Translation3d(0.0, omegaBallRadPerSec, 0.0)
                  .rotateBy(new Rotation3d(0.0, 0.0, turretRotation.getRadians()));
          params.omegaRadPerSec = omegaField;

          Logger.recordOutput("FuelVisualizer/Inputs/OmegaFieldX", omegaField.getX());
          Logger.recordOutput("FuelVisualizer/Inputs/OmegaFieldY", omegaField.getY());
          Logger.recordOutput("FuelVisualizer/Inputs/OmegaFieldZ", omegaField.getZ());

          return params;
        };

    return new VisualizeFuelShot(supplier);
  }

  public static class MotionParams {
    public Pose3d startPose;
    public Translation3d velocity;
    public Translation3d omegaRadPerSec;
  }

  private static final double kFuelDiameterMeters = 15.0 * 0.0254;
  private static final double kFuelRadiusMeters = kFuelDiameterMeters / 2.0;
  private static final double kFuelMassKg = 0.5 * 0.45359237;

  private static final double kFuelAreaM2 = Math.PI * kFuelRadiusMeters * kFuelRadiusMeters;

  private static final double kAirDensityKgPerM3 = 1.225;

  // Drag coefficient for a sphere
  private static final double kFuelCd = 0.55;

  // Magnus effect coefficients
  private static final double kMagnusClPerSpinRatio = 1.2;
  private static final double kMagnusClMax = 0.9;

  private static final double kGravity = 9.81;
  private static final double kEpsilon = 1e-6;

  private static final Random random = new Random();

  private static final ConcurrentHashMap<Integer, Pose3d> activeFuelShots =
      new ConcurrentHashMap<>();
  private static int nextFuelShotId = 0;

  protected Pose3d pose;
  protected final Supplier<MotionParams> paramsSupplier;
  protected MotionParams params;
  protected double lastTimestamp;
  protected final int fuelShotId;

  public VisualizeFuelShot(Supplier<MotionParams> paramsSupplier) {
    this.paramsSupplier = paramsSupplier;
    synchronized (VisualizeFuelShot.class) {
      this.fuelShotId = nextFuelShotId++;
    }
  }

  @Override
  public void initialize() {
    params = paramsSupplier.get();
    pose = params.startPose;
    lastTimestamp = Timer.getFPGATimestamp();

    activeFuelShots.put(fuelShotId, pose);

    Logger.recordOutput("FuelVisualizer/StartPose", params.startPose);
    Logger.recordOutput("FuelVisualizer/StartPoseX", params.startPose.getX());
    Logger.recordOutput("FuelVisualizer/StartPoseY", params.startPose.getY());
    Logger.recordOutput("FuelVisualizer/StartPoseZ", params.startPose.getZ());
    Logger.recordOutput("FuelVisualizer/InitialVelocity", params.velocity);
    Logger.recordOutput("FuelVisualizer/InitialOmega", params.omegaRadPerSec);
    Logger.recordOutput("FuelVisualizer/InitialVelocityX", params.velocity.getX());
    Logger.recordOutput("FuelVisualizer/InitialVelocityY", params.velocity.getY());
    Logger.recordOutput("FuelVisualizer/InitialVelocityZ", params.velocity.getZ());
    Logger.recordOutput("FuelVisualizer/InitialSpeed", params.velocity.getNorm());

    logAllActiveFuelShots();
  }

  @Override
  public void execute() {
    double timestamp = Timer.getFPGATimestamp();
    double dt = timestamp - lastTimestamp;
    if (dt <= 0.0) {
      return;
    }

    Translation3d v = params.velocity;
    double speed = v.getNorm();

    Translation3d accel = new Translation3d();

    if (speed > kEpsilon) {
      Translation3d vHat = v.div(speed);

      // q = 0.5 * rho * v^2
      double q = 0.5 * kAirDensityKgPerM3 * speed * speed;

      // Drag: Fd = q * Cd * A, opposite v
      double dragAccelMag = (q * kFuelCd * kFuelAreaM2) / kFuelMassKg;
      accel = accel.plus(vHat.times(-dragAccelMag));

      // Magnus: Fl = q * Cl * A, direction perpendicular to v and spin axis.
      Translation3d omega = params.omegaRadPerSec;
      double omegaMag = omega.getNorm();

      if (omegaMag > kEpsilon) {
        Translation3d omegaHat = omega.div(omegaMag);

        double spinRatio = (omegaMag * kFuelRadiusMeters) / speed;
        double cl = kMagnusClPerSpinRatio * spinRatio;
        cl = clamp(cl, -kMagnusClMax, kMagnusClMax);

        // Chosen so +Y backspin gives upward lift for a forward shot.
        Translation3d liftDir = cross(vHat, omegaHat);
        double liftDirMag = liftDir.getNorm();
        if (liftDirMag > kEpsilon) {
          liftDir = liftDir.div(liftDirMag);

          double magnusAccelMag = (q * cl * kFuelAreaM2) / kFuelMassKg;
          accel = accel.plus(liftDir.times(magnusAccelMag));
        }
      }
    }

    accel = accel.plus(new Translation3d(0.0, 0.0, -kGravity));

    params.velocity = params.velocity.plus(accel.times(dt));

    pose =
        new Pose3d(
            pose.getX() + params.velocity.getX() * dt,
            pose.getY() + params.velocity.getY() * dt,
            pose.getZ() + params.velocity.getZ() * dt,
            pose.getRotation());

    // Update this fuel shot's pose in the active collection
    activeFuelShots.put(fuelShotId, pose);

    // Log all active fuel shots
    logAllActiveFuelShots();

    Logger.recordOutput("FuelVisualizer/CurrentPoseX", pose.getX());
    Logger.recordOutput("FuelVisualizer/CurrentPoseY", pose.getY());
    Logger.recordOutput("FuelVisualizer/CurrentPoseZ", pose.getZ());
    Logger.recordOutput("FuelVisualizer/CurrentVelocity", params.velocity.getNorm());
    Logger.recordOutput("FuelVisualizer/CurrentVelocityX", params.velocity.getX());
    Logger.recordOutput("FuelVisualizer/CurrentVelocityY", params.velocity.getY());
    Logger.recordOutput("FuelVisualizer/CurrentVelocityZ", params.velocity.getZ());
    Logger.recordOutput("FuelVisualizer/TimeElapsed", timestamp - lastTimestamp);
    lastTimestamp = timestamp;
  }

  @Override
  public boolean isFinished() {
    boolean finished = false;
    String reason = "";

    if (pose.getZ() < 0.0) {
      finished = true;
      reason = "Z below ground";
    } else if (pose.getX() < 0.0) {
      finished = true;
      reason = "X out of bounds (negative)";
    } else if (pose.getX() > 20.0) {
      finished = true;
      reason = "X out of bounds (>20)";
    } else if (pose.getY() < 0.0) {
      finished = true;
      reason = "Y out of bounds (negative)";
    } else if (pose.getY() > 10.0) {
      finished = true;
      reason = "Y out of bounds (>10)";
    }

    if (finished) {
      Logger.recordOutput("FuelVisualizer/Finished", true);
      Logger.recordOutput("FuelVisualizer/FinishReason", reason);
      Logger.recordOutput("FuelVisualizer/FinalPoseX", pose.getX());
      Logger.recordOutput("FuelVisualizer/FinalPoseY", pose.getY());
      Logger.recordOutput("FuelVisualizer/FinalPoseZ", pose.getZ());
    } else {
      Logger.recordOutput("FuelVisualizer/Finished", false);
    }

    return finished;
  }

  @Override
  public void end(boolean interrupted) {
    activeFuelShots.remove(fuelShotId);
    logAllActiveFuelShots();
  }

  private static synchronized void logAllActiveFuelShots() {
    List<Pose3d> poses = new ArrayList<>(activeFuelShots.values());
    Logger.recordOutput("FuelVisualizer", poses.toArray(new Pose3d[0]));
  }

  private static double clamp(double x, double lo, double hi) {
    return Math.max(lo, Math.min(hi, x));
  }

  private static Translation3d cross(Translation3d a, Translation3d b) {
    return new Translation3d(
        a.getY() * b.getZ() - a.getZ() * b.getY(),
        a.getZ() * b.getX() - a.getX() * b.getZ(),
        a.getX() * b.getY() - a.getY() * b.getX());
  }
}
