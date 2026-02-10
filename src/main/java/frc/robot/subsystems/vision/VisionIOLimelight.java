package frc.robot.subsystems.vision;

import static frc.robot.subsystems.vision.VisionConstants.kLimelightFourCameraPose;
import static frc.robot.subsystems.vision.VisionConstants.kLimelightFourTable;
import static frc.robot.subsystems.vision.VisionConstants.kLimelightThreeCameraPose;
import static frc.robot.subsystems.vision.VisionConstants.kLimelightThreeTable;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.RobotState;
import frc.robot.util.LimelightHelpers;

public class VisionIOLimelight implements VisionIO {
  private final NetworkTable limelightFourTable;
  private final NetworkTable limelightThreeTable;
  private final RobotState robotState;

  public VisionIOLimelight() {
    limelightFourTable = NetworkTableInstance.getDefault().getTable(kLimelightFourTable);
    limelightThreeTable = NetworkTableInstance.getDefault().getTable(kLimelightThreeTable);
    robotState = RobotState.getInstance();
    configureLimelightSettings();
  }

  private void configureLimelightSettings() {
    limelightFourTable
        .getEntry("camerapose_robotspace_set")
        .setDoubleArray(kLimelightFourCameraPose);
    limelightThreeTable
        .getEntry("camerapose_robotspace_set")
        .setDoubleArray(kLimelightThreeCameraPose);
  }

  private void updateRobotOrientation() {
    var latestRobotPose = robotState.getLatestFieldToRobot();

    if (latestRobotPose != null) {
      Rotation2d robotRotation = latestRobotPose.getValue().getRotation();
      var robotSpeeds = robotState.getLatestRobotRelativeChassisSpeed();
      double robotYawRateDegPerS = Units.radiansToDegrees(robotSpeeds.omegaRadiansPerSecond);

      LimelightHelpers.SetRobotOrientation(
          kLimelightFourTable, robotRotation.getDegrees(), robotYawRateDegPerS, 0.0, 0.0, 0.0, 0.0);

      LimelightHelpers.SetRobotOrientation(
          kLimelightThreeTable,
          robotRotation.getDegrees(),
          robotYawRateDegPerS,
          0.0,
          0.0,
          0.0,
          0.0);
    }
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    inputs.connected = true;

    boolean limelightFourSeesTarget = limelightFourTable.getEntry("tv").getDouble(0) == 1.0;
    boolean limelightThreeSeesTarget = limelightThreeTable.getEntry("tv").getDouble(0) == 1.0;

    inputs.limelightFourHasTarget = limelightFourSeesTarget;
    inputs.limelightThreeHasTarget = limelightThreeSeesTarget;

    if (limelightFourSeesTarget) {
      var megatag2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(kLimelightFourTable);
      if (megatag2 != null && megatag2.tagCount > 0) {
        inputs.limelightFourPose = PoseObservation.fromLimelight(megatag2);
      } else {
        inputs.limelightFourPose = null;
      }
    } else {
      inputs.limelightFourPose = null;
    }

    if (limelightThreeSeesTarget) {
      var megatag2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(kLimelightThreeTable);
      if (megatag2 != null && megatag2.tagCount > 0) {
        inputs.limelightThreePose = PoseObservation.fromLimelight(megatag2);
      } else {
        inputs.limelightThreePose = null;
      }
    } else {
      inputs.limelightThreePose = null;
    }

    updateRobotOrientation();
  }
}
