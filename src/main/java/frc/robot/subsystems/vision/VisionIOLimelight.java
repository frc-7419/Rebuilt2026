package frc.robot.subsystems.vision;

import static frc.robot.subsystems.vision.VisionConstants.kLeftCameraPose;
import static frc.robot.subsystems.vision.VisionConstants.kLeftCameraTable;
import static frc.robot.subsystems.vision.VisionConstants.kRightCameraPose;
import static frc.robot.subsystems.vision.VisionConstants.kRightCameraTable;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.RobotState;
import frc.robot.util.LimelightHelpers;
import java.util.concurrent.atomic.AtomicReference;

public class VisionIOLimelight implements VisionIO {
  private final NetworkTable leftTable;
  private final NetworkTable rightTable;
  private final RobotState robotState;
  private final AtomicReference<VisionIOInputs> cachedInputs =
      new AtomicReference<>(new VisionIOInputs());

  public VisionIOLimelight() {
    leftTable = NetworkTableInstance.getDefault().getTable(kLeftCameraTable);
    rightTable = NetworkTableInstance.getDefault().getTable(kRightCameraTable);
    robotState = RobotState.getInstance();
    configureLimelightSettings();
  }

  private void configureLimelightSettings() {
    leftTable.getEntry("camerapose_robotspace_set").setDoubleArray(kLeftCameraPose);
    rightTable.getEntry("camerapose_robotspace_set").setDoubleArray(kRightCameraPose);
  }

  private void updateRobotOrientation() {
    var latestRobotPose = robotState.getLatestFieldToRobot();

    if (latestRobotPose != null) {
      Rotation2d robotRotation = latestRobotPose.getValue().getRotation();
      var robotSpeeds = robotState.getLatestRobotRelativeChassisSpeed();
      double robotYawRateDegPerS = Units.radiansToDegrees(robotSpeeds.omegaRadiansPerSecond);

      LimelightHelpers.SetRobotOrientation(
          kLeftCameraTable, robotRotation.getDegrees(), robotYawRateDegPerS, 0.0, 0.0, 0.0, 0.0);

      LimelightHelpers.SetRobotOrientation(
          kRightCameraTable, robotRotation.getDegrees(), robotYawRateDegPerS, 0.0, 0.0, 0.0, 0.0);
    }
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    inputs.connected = true;

    boolean leftSeesTarget = leftTable.getEntry("tv").getDouble(0) == 1.0;
    boolean rightSeesTarget = rightTable.getEntry("tv").getDouble(0) == 1.0;

    inputs.leftHasTarget = leftSeesTarget;
    inputs.rightHasTarget = rightSeesTarget;

    if (leftSeesTarget) {
      var megatag2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(kLeftCameraTable);
      if (megatag2 != null && megatag2.tagCount > 0) {
        inputs.leftPose = PoseObservation.fromLimelight(megatag2);
      } else {
        inputs.leftPose = null;
      }
    } else {
      inputs.leftPose = null;
    }

    if (rightSeesTarget) {
      var megatag2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(kRightCameraTable);
      if (megatag2 != null && megatag2.tagCount > 0) {
        inputs.rightPose = PoseObservation.fromLimelight(megatag2);
      } else {
        inputs.rightPose = null;
      }
    } else {
      inputs.rightPose = null;
    }

    cachedInputs.set(inputs);
    updateRobotOrientation();
  }
}
