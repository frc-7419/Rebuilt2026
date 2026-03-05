package frc.robot.subsystems.vision;

import static frc.robot.subsystems.vision.VisionConstants.kLimelightFourCameraPose;
import static frc.robot.subsystems.vision.VisionConstants.kLimelightFourTable;
import static frc.robot.subsystems.vision.VisionConstants.kLimelightThreeCameraPose;
import static frc.robot.subsystems.vision.VisionConstants.kLimelightThreeTable;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.RobotState;
import frc.robot.util.LimelightHelpers;

public class VisionIOLimelight implements VisionIO {
  private static final double kConnectionTimeoutSec = 0.5;

  private final NetworkTable limelightFourTable;
  private final NetworkTable limelightThreeTable;
  private final RobotState robotState;

  private long lastChangeFour = 0;
  private long lastChangeThree = 0;
  private double lastConnectedFourSec = 0;
  private double lastConnectedThreeSec = 0;

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
    if (latestRobotPose == null) return;

    Rotation2d robotRotation = latestRobotPose.getValue().getRotation();
    var robotSpeeds = robotState.getLatestRobotRelativeChassisSpeed();
    double robotYawRateDegPerS = Units.radiansToDegrees(robotSpeeds.omegaRadiansPerSecond);

    LimelightHelpers.SetRobotOrientation(
        kLimelightFourTable, robotRotation.getDegrees(), robotYawRateDegPerS, 0.0, 0.0, 0.0, 0.0);
    LimelightHelpers.SetRobotOrientation(
        kLimelightThreeTable, robotRotation.getDegrees(), robotYawRateDegPerS, 0.0, 0.0, 0.0, 0.0);
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    updateRobotOrientation();

    double nowSec = Timer.getFPGATimestamp();
    long changeFour = limelightFourTable.getEntry("tl").getLastChange();
    long changeThree = limelightThreeTable.getEntry("tl").getLastChange();
    if (changeFour != lastChangeFour) {
      lastChangeFour = changeFour;
      lastConnectedFourSec = nowSec;
    }
    if (changeThree != lastChangeThree) {
      lastChangeThree = changeThree;
      lastConnectedThreeSec = nowSec;
    }
    inputs.leftConnected = (nowSec - lastConnectedFourSec) < kConnectionTimeoutSec;
    inputs.rightConnected = (nowSec - lastConnectedThreeSec) < kConnectionTimeoutSec;

    boolean limelightFourSeesTarget = limelightFourTable.getEntry("tv").getDouble(0) == 1.0;
    boolean limelightThreeSeesTarget = limelightThreeTable.getEntry("tv").getDouble(0) == 1.0;

    inputs.limelightFourHasTarget = limelightFourSeesTarget;
    inputs.limelightThreeHasTarget = limelightThreeSeesTarget;

    if (limelightFourSeesTarget) {
      var mt1 = LimelightHelpers.getBotPoseEstimate_wpiBlue(kLimelightFourTable);
      inputs.limelightFourMT1Pose =
          (mt1 != null && mt1.tagCount > 0) ? PoseObservation.fromLimelight(mt1) : null;

      var mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(kLimelightFourTable);
      inputs.limelightFourMT2Pose =
          (mt2 != null && mt2.tagCount > 0) ? PoseObservation.fromLimelight(mt2) : null;
    } else {
      inputs.limelightFourMT1Pose = null;
      inputs.limelightFourMT2Pose = null;
    }

    if (limelightThreeSeesTarget) {
      var mt1 = LimelightHelpers.getBotPoseEstimate_wpiBlue(kLimelightThreeTable);
      inputs.limelightThreeMT1Pose =
          (mt1 != null && mt1.tagCount > 0) ? PoseObservation.fromLimelight(mt1) : null;

      var mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(kLimelightThreeTable);
      inputs.limelightThreeMT2Pose =
          (mt2 != null && mt2.tagCount > 0) ? PoseObservation.fromLimelight(mt2) : null;
    } else {
      inputs.limelightThreeMT1Pose = null;
      inputs.limelightThreeMT2Pose = null;
    }
  }
}
