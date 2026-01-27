package frc.robot.subsystems.vision;

import static frc.robot.subsystems.vision.VisionConstants.kLeftCameraPose;
import static frc.robot.subsystems.vision.VisionConstants.kRightCameraPose;

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
    Logger.recordOutput("Vision/leftHasTarget", inputs.leftHasTarget);
    Logger.recordOutput("Vision/rightHasTarget", inputs.rightHasTarget);

    logCameraPoses();

    if (inputs.leftHasTarget && inputs.leftPose != null) {
      updateVision(inputs.leftPose, true);
    }
    if (inputs.rightHasTarget && inputs.rightPose != null) {
      updateVision(inputs.rightPose, false);
    }

    Logger.recordOutput("Vision/latencyPeriodicSec", Timer.getFPGATimestamp() - timestamp);
  }

  @AutoLogOutput(key = "Vision/LeftCameraPose")
  private Pose3d leftCameraPose = new Pose3d();

  @AutoLogOutput(key = "Vision/RightCameraPose")
  private Pose3d rightCameraPose = new Pose3d();

  private void logCameraPoses() {
    var latestRobotPose = robotState.getLatestFieldToRobot();
    if (latestRobotPose == null) {
      leftCameraPose = new Pose3d();
      rightCameraPose = new Pose3d();
      return;
    }

    Pose2d robotPose = latestRobotPose.getValue();
    Pose3d robotPose3d = new Pose3d(robotPose);

    Transform3d robotToLeftCamera3d = getCameraTransform(true);
    leftCameraPose = robotPose3d.transformBy(robotToLeftCamera3d);

    Transform3d robotToRightCamera3d = getCameraTransform(false);
    rightCameraPose = robotPose3d.transformBy(robotToRightCamera3d);
  }

  private void updateVision(PoseObservation observation, boolean isLeftCamera) {
    String logPrefix = "Vision/" + (isLeftCamera ? "Left/" : "Right/");
    double timestamp = observation.timestampSeconds;
    double lastProcessedTimestamp =
        isLeftCamera ? lastProcessedLeftTimestamp : lastProcessedRightTimestamp;

    if (timestamp == lastProcessedTimestamp) {
      return;
    }

    Optional<RobotState.VisionObservation> megatag2Estimate =
        processMegatag2Estimate(observation, isLeftCamera, logPrefix);

    if (megatag2Estimate.isPresent()) {
      if (shouldUseMegatag2(observation, isLeftCamera, logPrefix)) {
        Logger.recordOutput(logPrefix + "Megatag2Estimate", megatag2Estimate.get().visionPose());
        addVisionObservation(megatag2Estimate.get());
      } else {
        Logger.recordOutput(
            logPrefix + "Megatag2EstimateRejected", megatag2Estimate.get().visionPose());
      }
    }

    if (isLeftCamera) {
      lastProcessedLeftTimestamp = timestamp;
    } else {
      lastProcessedRightTimestamp = timestamp;
    }
  }

  private boolean shouldUseMegatag2(
      PoseObservation observation, boolean isLeftCamera, String logPrefix) {
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
      PoseObservation observation, boolean isLeftCamera) {
    Pose2d fieldToCamera = observation.estimatedPose;
    if (fieldToCamera.getX() == 0.0) {
      return Optional.empty();
    }

    Transform3d robotToCamera3d = getCameraTransform(isLeftCamera);
    Transform3d cameraToRobot3d = robotToCamera3d.inverse();
    Transform2d cameraToRobot2d =
        new Transform2d(
            new Translation2d(cameraToRobot3d.getX(), cameraToRobot3d.getY()),
            new Rotation2d(cameraToRobot3d.getRotation().getZ()));
    return Optional.of(fieldToCamera.transformBy(cameraToRobot2d));
  }

  private Transform3d getCameraTransform(boolean isLeftCamera) {
    if (isLeftCamera) {
      // kLeftCameraPose: [forward, side, up, roll(deg), pitch(deg), yaw(deg)]
      return new Transform3d(
          new Translation3d(kLeftCameraPose[0], kLeftCameraPose[1], kLeftCameraPose[2]),
          new Rotation3d(
              Units.degreesToRadians(kLeftCameraPose[3]),
              Units.degreesToRadians(kLeftCameraPose[4]),
              Units.degreesToRadians(kLeftCameraPose[5])));
    } else {
      // kRightCameraPose: [forward, side, up, roll(deg), pitch(deg), yaw(deg)]
      return new Transform3d(
          new Translation3d(kRightCameraPose[0], kRightCameraPose[1], kRightCameraPose[2]),
          new Rotation3d(
              Units.degreesToRadians(kRightCameraPose[3]),
              Units.degreesToRadians(kRightCameraPose[4]),
              Units.degreesToRadians(kRightCameraPose[5])));
    }
  }

  private Optional<RobotState.VisionObservation> processMegatag2Estimate(
      PoseObservation observation, boolean isLeftCamera, String logPrefix) {
    Optional<Pose2d> loggedFieldToRobot = robotState.getFieldToRobot(observation.timestampSeconds);
    if (loggedFieldToRobot.isEmpty()) {
      return Optional.empty();
    }

    Optional<Pose2d> fieldToRobotEstimate = getFieldToRobotEstimate(observation, isLeftCamera);
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
      drive.addVisionMeasurement(
          observation.visionPose(), observation.timestamp(), observation.stdDevs());
    }
  }
}
