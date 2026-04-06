package frc.robot.subsystems.vision;

import static frc.robot.subsystems.vision.VisionConstants.kLimelightFourCameraPose;
import static frc.robot.subsystems.vision.VisionConstants.kLimelightThreeCameraPose;
import static frc.robot.subsystems.vision.VisionConstants.kLimelightTwoCameraPose;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import frc.robot.RobotState;
import frc.robot.subsystems.drive.Drive;
import java.util.Optional;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;

public class Vision extends SubsystemBase {
  private static final String LOG_FOUR = "Vision/LimelightFour/";
  private static final String LOG_THREE = "Vision/LimelightThree/";
  private static final String LOG_TWO = "Vision/LimelightTwo/";

  private static final LoggedNetworkBoolean overrideMegatag1Only =
      new LoggedNetworkBoolean("Vision/OverrideMegatag1Only", false);

  private final VisionIO io;
  private final RobotState robotState;
  private final VisionIO.VisionIOInputs inputs = new VisionIO.VisionIOInputs();
  private Drive drive;

  private boolean useMegatag2Mode = false;
  private final Alert leftVisionDisconnectedAlert =
      new Alert("Disconnected left vision camera (limelight-four).", AlertType.kError);
  private final Alert rightVisionDisconnectedAlert =
      new Alert("Disconnected right vision camera (limelight-three).", AlertType.kError);
  private final Alert rearVisionDisconnectedAlert =
      new Alert("Disconnected rear vision camera (limelight-two).", AlertType.kError);

  private double lastFourMT1Timestamp = 0.0;
  private double lastFourMT2Timestamp = 0.0;
  private double lastThreeMT1Timestamp = 0.0;
  private double lastThreeMT2Timestamp = 0.0;
  private double lastTwoMT1Timestamp = 0.0;
  private double lastTwoMT2Timestamp = 0.0;

  public Vision(VisionIO io) {
    this.io = io;
    this.robotState = RobotState.getInstance();
  }

  public void setDrive(Drive drive) {
    this.drive = drive;
  }

  public boolean isMegatag2() {
    return useMegatag2Mode;
  }

  public void useMegatag1() {
    useMegatag2Mode = false;
  }

  public void useMegatag2() {
    useMegatag2Mode = true;
  }

  @Override
  public void periodic() {
    double startTime = Timer.getFPGATimestamp();
    io.updateInputs(inputs);

    Logger.recordOutput("Vision/leftConnected", inputs.leftConnected);
    Logger.recordOutput("Vision/rightConnected", inputs.rightConnected);
    Logger.recordOutput("Vision/rearConnected", inputs.rearConnected);
    robotState.setVisionDeviceConnections(
        inputs.leftConnected, inputs.rightConnected, inputs.rearConnected);
    if (Constants.currentMode != Mode.SIM) {
      leftVisionDisconnectedAlert.set(!inputs.leftConnected);
      rightVisionDisconnectedAlert.set(!inputs.rightConnected);
      rearVisionDisconnectedAlert.set(!inputs.rearConnected);
    }
    Logger.recordOutput("Vision/limelightFourHasTarget", inputs.limelightFourHasTarget);
    Logger.recordOutput("Vision/limelightThreeHasTarget", inputs.limelightThreeHasTarget);
    Logger.recordOutput("Vision/limelightTwoHasTarget", inputs.limelightTwoHasTarget);
    Logger.recordOutput("Vision/useMegatag2Mode", useMegatag2Mode);
    Logger.recordOutput("Vision/overrideMegatag1Only", overrideMegatag1Only.get());

    logCameraPoses();

    boolean useMT2 = useMegatag2Mode && !overrideMegatag1Only.get();
    Logger.recordOutput("Vision/effectiveMegatag2", useMT2);
    if (useMT2) {
      if (inputs.limelightFourMT2Pose != null
          && inputs.limelightFourMT2Pose.timestampSeconds != lastFourMT2Timestamp) {
        processMegatag2(inputs.limelightFourMT2Pose, LOG_FOUR);
        lastFourMT2Timestamp = inputs.limelightFourMT2Pose.timestampSeconds;
      }
      if (inputs.limelightThreeMT2Pose != null
          && inputs.limelightThreeMT2Pose.timestampSeconds != lastThreeMT2Timestamp) {
        processMegatag2(inputs.limelightThreeMT2Pose, LOG_THREE);
        lastThreeMT2Timestamp = inputs.limelightThreeMT2Pose.timestampSeconds;
      }
      if (inputs.limelightTwoMT2Pose != null
          && inputs.limelightTwoMT2Pose.timestampSeconds != lastTwoMT2Timestamp) {
        processMegatag2(inputs.limelightTwoMT2Pose, LOG_TWO);
        lastTwoMT2Timestamp = inputs.limelightTwoMT2Pose.timestampSeconds;
      }
    } else {
      if (inputs.limelightFourMT1Pose != null
          && inputs.limelightFourMT1Pose.timestampSeconds != lastFourMT1Timestamp) {
        processMegatag1(inputs.limelightFourMT1Pose, LOG_FOUR);
        lastFourMT1Timestamp = inputs.limelightFourMT1Pose.timestampSeconds;
      }
      if (inputs.limelightThreeMT1Pose != null
          && inputs.limelightThreeMT1Pose.timestampSeconds != lastThreeMT1Timestamp) {
        processMegatag1(inputs.limelightThreeMT1Pose, LOG_THREE);
        lastThreeMT1Timestamp = inputs.limelightThreeMT1Pose.timestampSeconds;
      }
      if (inputs.limelightTwoMT1Pose != null
          && inputs.limelightTwoMT1Pose.timestampSeconds != lastTwoMT1Timestamp) {
        processMegatag1(inputs.limelightTwoMT1Pose, LOG_TWO);
        lastTwoMT1Timestamp = inputs.limelightTwoMT1Pose.timestampSeconds;
      }
    }

    Logger.recordOutput("Vision/latencyPeriodicSec", Timer.getFPGATimestamp() - startTime);
  }

  @AutoLogOutput(key = "Vision/LimelightFourPose")
  private Pose3d limelightFourCamPose = new Pose3d();

  @AutoLogOutput(key = "Vision/LimelightThreePose")
  private Pose3d limelightThreeCamPose = new Pose3d();

  @AutoLogOutput(key = "Vision/LimelightTwoPose")
  private Pose3d limelightTwoCamPose = new Pose3d();

  private void logCameraPoses() {
    var latestRobotPose = robotState.getLatestFieldToRobot();
    if (latestRobotPose == null) {
      limelightFourCamPose = new Pose3d();
      limelightThreeCamPose = new Pose3d();
      limelightTwoCamPose = new Pose3d();
      return;
    }
    Pose3d robotPose3d = new Pose3d(latestRobotPose.getValue());
    limelightFourCamPose = robotPose3d.transformBy(getCameraTransform(0));
    limelightThreeCamPose = robotPose3d.transformBy(getCameraTransform(1));
    limelightTwoCamPose = robotPose3d.transformBy(getCameraTransform(2));
  }

  private void processMegatag1(PoseObservation observation, String logPrefix) {
    if (!hasValidPose(observation)) return;
    if (!isMotionAcceptable(logPrefix)) {
      Logger.recordOutput(logPrefix + "MT1Rejected", observation.estimatedPose);
      return;
    }

    double xyStdDevMeters;
    double thetaStdDevRad;
    if (observation.tagCount >= 2) {
      xyStdDevMeters = 0.75;
      thetaStdDevRad = Units.degreesToRadians(10.0);
    } else {
      xyStdDevMeters = 3;
      thetaStdDevRad = Units.degreesToRadians(30.0);
    }

    Matrix<N3, N1> stdDevs = VecBuilder.fill(xyStdDevMeters, xyStdDevMeters, thetaStdDevRad);
    Logger.recordOutput(logPrefix + "MT1Estimate", observation.estimatedPose);
    Logger.recordOutput(logPrefix + "MT1XYStdDevMeters", xyStdDevMeters);

    addVisionObservation(
        new RobotState.VisionObservation(
            observation.timestampSeconds, observation.estimatedPose, stdDevs));
  }

  private void processMegatag2(PoseObservation observation, String logPrefix) {
    if (!hasValidPose(observation)) return;
    if (!isMotionAcceptable(logPrefix)) {
      Logger.recordOutput(logPrefix + "MT2Rejected", observation.estimatedPose);
      return;
    }

    Optional<Pose2d> odometryPoseAtTime = robotState.getFieldToRobot(observation.timestampSeconds);
    if (odometryPoseAtTime.isEmpty()) return;

    double poseDifferenceMeters =
        observation
            .estimatedPose
            .getTranslation()
            .getDistance(odometryPoseAtTime.get().getTranslation());

    double xyStdDevMeters = (observation.tagCount >= 2) ? 0.2 : 0.8;

    if (poseDifferenceMeters < 0.20) {
      xyStdDevMeters *= 2.0;
    } else if (poseDifferenceMeters < 0.50) {
      xyStdDevMeters *= 1.5;
    } else if (poseDifferenceMeters > 1.50) {
      xyStdDevMeters *= 3;
    }

    Pose2d correctedPose =
        new Pose2d(
            observation.estimatedPose.getTranslation(), odometryPoseAtTime.get().getRotation());

    Matrix<N3, N1> stdDevs =
        VecBuilder.fill(xyStdDevMeters, xyStdDevMeters, Units.degreesToRadians(9999999.0));

    Logger.recordOutput(logPrefix + "MT2Estimate", correctedPose);
    Logger.recordOutput(logPrefix + "MT2XYStdDevMeters", xyStdDevMeters);
    Logger.recordOutput(logPrefix + "MT2PoseDifferenceMeters", poseDifferenceMeters);

    addVisionObservation(
        new RobotState.VisionObservation(observation.timestampSeconds, correctedPose, stdDevs));
  }

  private boolean hasValidPose(PoseObservation observation) {
    return observation.estimatedPose.getX() != 0.0 || observation.estimatedPose.getY() != 0.0;
  }

  private boolean isMotionAcceptable(String logPrefix) {
    final double kMaxYawRateRadPerS = Units.degreesToRadians(360.0);
    double yawRate =
        Math.abs(robotState.getLatestRobotRelativeChassisSpeed().omegaRadiansPerSecond);
    boolean acceptable = yawRate <= kMaxYawRateRadPerS;
    Logger.recordOutput(logPrefix + "motionAcceptable", acceptable);
    return acceptable;
  }

  private Transform3d getCameraTransform(int cameraIndex) {
    double[] p;
    switch (cameraIndex) {
      case 0:
        p = kLimelightFourCameraPose;
        break;
      case 1:
        p = kLimelightThreeCameraPose;
        break;
      case 2:
        p = kLimelightTwoCameraPose;
        break;
      default:
        throw new IllegalArgumentException("Invalid camera index: " + cameraIndex);
    }
    return new Transform3d(
        new Translation3d(p[0], p[1], p[2]),
        new Rotation3d(
            Units.degreesToRadians(p[3]),
            Units.degreesToRadians(p[4]),
            Units.degreesToRadians(p[5])));
  }

  private void addVisionObservation(RobotState.VisionObservation observation) {
    if (drive != null) {
      drive.addVisionMeasurement(
          observation.visionPose(), observation.timestamp(), observation.stdDevs());
    }
  }
}
