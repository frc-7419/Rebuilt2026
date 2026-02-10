package frc.robot.subsystems.vision;

import static frc.robot.subsystems.vision.VisionConstants.kLimelightFourCameraPose;
import static frc.robot.subsystems.vision.VisionConstants.kLimelightThreeCameraPose;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotState;
import frc.robot.subsystems.drive.Drive;
import java.util.Optional;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Vision extends SubsystemBase {
  private final VisionIO io;
  private final RobotState robotState;
  private final VisionIO.VisionIOInputs inputs = new VisionIO.VisionIOInputs();
  private Drive drive;

  private double lastProcessedLeftTimestamp = 0.0;
  private double lastProcessedRightTimestamp = 0.0;

  public Vision(VisionIO io) {
    this.io = io;
    this.robotState = RobotState.getInstance();
  }

  public void setDrive(Drive drive) {
    this.drive = drive;
  }

  @Override
  public void periodic() {
    double timestamp = Timer.getFPGATimestamp();
    io.updateInputs(inputs);
    Logger.recordOutput("Vision/connected", inputs.connected);
    Logger.recordOutput("Vision/limelightFourHasTarget", inputs.limelightFourHasTarget);
    Logger.recordOutput("Vision/limelightThreeHasTarget", inputs.limelightThreeHasTarget);

    logCameraPoses();

    if (inputs.limelightFourHasTarget && inputs.limelightFourPose != null) {
      updateVision(inputs.limelightFourPose, true);
    }
    if (inputs.limelightThreeHasTarget && inputs.limelightThreePose != null) {
      updateVision(inputs.limelightThreePose, false);
    }

    Logger.recordOutput("Vision/latencyPeriodicSec", Timer.getFPGATimestamp() - timestamp);
  }

  @AutoLogOutput(key = "Vision/LimelightFourPose")
  private Pose3d limelightFourPose = new Pose3d();

  @AutoLogOutput(key = "Vision/LimelightThreePose")
  private Pose3d limelightThreePose = new Pose3d();

  private void logCameraPoses() {
    var latestRobotPose = robotState.getLatestFieldToRobot();
    if (latestRobotPose == null) {
      limelightFourPose = new Pose3d();
      limelightThreePose = new Pose3d();
      return;
    }

    Pose2d robotPose = latestRobotPose.getValue();
    Pose3d robotPose3d = new Pose3d(robotPose);

    Transform3d robotToLimelightFourCamera3d = getCameraTransform(true);
    limelightFourPose = robotPose3d.transformBy(robotToLimelightFourCamera3d);

    Transform3d robotToLimelightThreeCamera3d = getCameraTransform(false);
    limelightThreePose = robotPose3d.transformBy(robotToLimelightThreeCamera3d);
  }

  private void updateVision(PoseObservation observation, boolean isLimelightFour) {
    String logPrefix = "Vision/" + (isLimelightFour ? "LimelightFour/" : "LimelightThree/");
    double timestamp = observation.timestampSeconds;
    double lastProcessedTimestamp =
        isLimelightFour ? lastProcessedLeftTimestamp : lastProcessedRightTimestamp;

    if (timestamp == lastProcessedTimestamp) {
      return;
    }

    Optional<RobotState.VisionObservation> megatag2Estimate =
        processMegatag2Estimate(observation, isLimelightFour, logPrefix);

    if (megatag2Estimate.isPresent()) {
      if (shouldUseMegatag2(observation, isLimelightFour, logPrefix)) {
        Logger.recordOutput(logPrefix + "Megatag2Estimate", megatag2Estimate.get().visionPose());
        addVisionObservation(megatag2Estimate.get());
      } else {
        Logger.recordOutput(
            logPrefix + "Megatag2EstimateRejected", megatag2Estimate.get().visionPose());
      }
    }

    if (isLimelightFour) {
      lastProcessedLeftTimestamp = timestamp;
    } else {
      lastProcessedRightTimestamp = timestamp;
    }
  }

  private boolean shouldUseMegatag2(
      PoseObservation observation, boolean isLimelightFour, String logPrefix) {
    return isMotionAcceptable(observation.timestampSeconds, logPrefix);
  }

  private boolean isMotionAcceptable(double timestamp, String logPrefix) {
    final double kMaxYawRateRadPerS = Units.degreesToRadians(100.0);

    var robotSpeeds = robotState.getLatestRobotRelativeChassisSpeed();
    double robotYawRateRadPerS = Math.abs(robotSpeeds.omegaRadiansPerSecond);

    if (robotYawRateRadPerS > kMaxYawRateRadPerS) {
      Logger.recordOutput(logPrefix + "motionAcceptable", false);
      return false;
    }
    Logger.recordOutput(logPrefix + "motionAcceptable", true);
    return true;
  }

  private Optional<Pose2d> getFieldToRobotEstimate(
      PoseObservation observation, boolean isLimelightFour) {
    Pose2d fieldToCamera = observation.estimatedPose;
    if (fieldToCamera.getX() == 0.0) {
      return Optional.empty();
    }

    Transform3d robotToCamera3d = getCameraTransform(isLimelightFour);
    Transform3d cameraToRobot3d = robotToCamera3d.inverse();
    Transform2d cameraToRobot2d =
        new Transform2d(
            new Translation2d(cameraToRobot3d.getX(), cameraToRobot3d.getY()),
            new Rotation2d(cameraToRobot3d.getRotation().getZ()));
    return Optional.of(fieldToCamera.transformBy(cameraToRobot2d));
  }

  private Transform3d getCameraTransform(boolean isLimelightFour) {
    if (isLimelightFour) {
      // kLeftCameraPose: [forward, side, up, roll(deg), pitch(deg), yaw(deg)]
      return new Transform3d(
          new Translation3d(
              kLimelightFourCameraPose[0],
              kLimelightFourCameraPose[1],
              kLimelightFourCameraPose[2]),
          new Rotation3d(
              Units.degreesToRadians(kLimelightFourCameraPose[3]),
              Units.degreesToRadians(kLimelightFourCameraPose[4]),
              Units.degreesToRadians(kLimelightFourCameraPose[5])));
    } else {
      // kRightCameraPose: [forward, side, up, roll(deg), pitch(deg), yaw(deg)]
      return new Transform3d(
          new Translation3d(
              kLimelightThreeCameraPose[0],
              kLimelightThreeCameraPose[1],
              kLimelightThreeCameraPose[2]),
          new Rotation3d(
              Units.degreesToRadians(kLimelightThreeCameraPose[3]),
              Units.degreesToRadians(kLimelightThreeCameraPose[4]),
              Units.degreesToRadians(kLimelightThreeCameraPose[5])));
    }
  }

  private Optional<RobotState.VisionObservation> processMegatag2Estimate(
      PoseObservation observation, boolean isLimelightFour, String logPrefix) {
    Optional<Pose2d> loggedFieldToRobot = robotState.getFieldToRobot(observation.timestampSeconds);
    if (loggedFieldToRobot.isEmpty()) {
      return Optional.empty();
    }

    Optional<Pose2d> fieldToRobotEstimate = getFieldToRobotEstimate(observation, isLimelightFour);
    if (fieldToRobotEstimate.isEmpty()) {
      return Optional.empty();
    }

    double poseDifferenceMeters =
        fieldToRobotEstimate
            .get()
            .getTranslation()
            .getDistance(loggedFieldToRobot.get().getTranslation());

    double xyStdDevMeters;
    if (observation.fiducialIds != null && observation.fiducialIds.length > 0) {
      if (observation.tagCount >= 2 || observation.fiducialIds.length >= 2) {
        xyStdDevMeters = 0.2;
      } else {
        xyStdDevMeters = 0.80;
      }

      if (poseDifferenceMeters < 0.20) {
        xyStdDevMeters *= 2.0;
      } else if (poseDifferenceMeters < 0.50) {
        xyStdDevMeters *= 1.3;
      } else if (poseDifferenceMeters > 1.50) {
        xyStdDevMeters *= 0.75;
      }

      Logger.recordOutput(logPrefix + "megatag2StdDevMeters", xyStdDevMeters);
      Logger.recordOutput(logPrefix + "megatag2PoseDifferenceMeters", poseDifferenceMeters);

      double thetaStdDevRad;
      if (observation.tagCount >= 2 || observation.fiducialIds.length >= 2) {
        thetaStdDevRad = Units.degreesToRadians(15.0);
      } else {
        thetaStdDevRad = Units.degreesToRadians(30.0);
      }

      Matrix<N3, N1> stdDevs = VecBuilder.fill(xyStdDevMeters, xyStdDevMeters, thetaStdDevRad);
      Pose2d correctedPose =
          new Pose2d(
              fieldToRobotEstimate.get().getTranslation(), loggedFieldToRobot.get().getRotation());
      return Optional.of(
          new RobotState.VisionObservation(observation.timestampSeconds, correctedPose, stdDevs));
    }
    return Optional.empty();
  }

  private void addVisionObservation(RobotState.VisionObservation observation) {
    if (drive != null) {
      /*   Pose2d adjusted =
          new Pose2d(
              observation.visionPose().getX(),
              observation.visionPose().getY(),
              observation.visionPose().getRotation().rotateBy(Rotation2d.fromDegrees(180)));
      Logger.recordOutput("Adjusted Pose", adjusted);*/
      drive.addVisionMeasurement(
          observation.visionPose(), observation.timestamp(), observation.stdDevs());
    }
  }
}
