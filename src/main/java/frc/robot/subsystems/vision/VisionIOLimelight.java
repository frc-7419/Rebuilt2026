package frc.robot.subsystems.vision;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.RobotState;
import frc.robot.util.LimelightHelpers;
import java.util.concurrent.atomic.AtomicReference;

import static frc.robot.subsystems.vision.VisionConstants.kTurretCameraTable;
import static frc.robot.subsystems.vision.VisionConstants.kSupplementaryCameraTable;
import static frc.robot.subsystems.vision.VisionConstants.kTurretCameraPose;
import static frc.robot.subsystems.vision.VisionConstants.kSupplementaryCameraPose;

public class VisionIOLimelight implements VisionIO {
  private final NetworkTable turretTable;
  private final NetworkTable supplementaryTable;
  private final RobotState robotState;
  private final AtomicReference<VisionIOInputs> cachedInputs = new AtomicReference<>(new VisionIOInputs());

  public VisionIOLimelight() {
    turretTable = NetworkTableInstance.getDefault().getTable(kTurretCameraTable);
    supplementaryTable = NetworkTableInstance.getDefault().getTable(kSupplementaryCameraTable);
    robotState = RobotState.getInstance();
    configureLimelightSettings();
  }

  private void configureLimelightSettings() {
    turretTable.getEntry("camerapose_robotspace_set").setDoubleArray(kTurretCameraPose);
    supplementaryTable.getEntry("camerapose_robotspace_set").setDoubleArray(kSupplementaryCameraPose);
  }

  private void updateRobotOrientation() {
    var latestRobotPose = robotState.getLatestFieldToRobot();
    var latestTurretRotation = robotState.getLatestRobotToTurret();

    if (latestRobotPose != null && latestTurretRotation != null) {
      Rotation2d fieldToTurretRotation =
          latestRobotPose.getValue().getRotation().plus(latestTurretRotation.getValue());
      var robotSpeeds = robotState.getLatestRobotRelativeChassisSpeed();
      var turretAngularVelocityMeasure = robotState.getLatestTurretAngularVelocity();
      double turretAngularVelocityRadPerS = turretAngularVelocityMeasure.in(RadiansPerSecond);
      double combinedYawRateRadPerS = robotSpeeds.omegaRadiansPerSecond + turretAngularVelocityRadPerS;
      double combinedYawRateDegPerS = Units.radiansToDegrees(combinedYawRateRadPerS);

      LimelightHelpers.SetRobotOrientation(
          kTurretCameraTable,
          fieldToTurretRotation.getDegrees(),
          combinedYawRateDegPerS,
          0.0, 0.0, 0.0, 0.0);

      Rotation2d robotRotation = latestRobotPose.getValue().getRotation();
      double robotYawRateDegPerS = Units.radiansToDegrees(robotSpeeds.omegaRadiansPerSecond);

      LimelightHelpers.SetRobotOrientation(
          kSupplementaryCameraTable,
          robotRotation.getDegrees(),
          robotYawRateDegPerS,
          0.0, 0.0, 0.0, 0.0);
    }
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    inputs.connected = true;

    boolean turretSeesTarget = turretTable.getEntry("tv").getDouble(0) == 1.0;
    boolean supplementarySeesTarget = supplementaryTable.getEntry("tv").getDouble(0) == 1.0;

    inputs.turretHasTarget = turretSeesTarget;
    inputs.supplementaryHasTarget = supplementarySeesTarget;

    if (turretSeesTarget) {
      var megatag1 = LimelightHelpers.getBotPoseEstimate_wpiBlue(kTurretCameraTable);
      var megatag2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(kTurretCameraTable);

      if (megatag1.tagCount > 0) {
        inputs.turretPose = PoseObservation.fromLimelight(megatag1);
      } else if (megatag2.tagCount > 0) {
        inputs.turretPose = PoseObservation.fromLimelight(megatag2);
      } else {
        inputs.turretPose = null;
      }
    } else {
      inputs.turretPose = null;
    }

    if (supplementarySeesTarget) {
      var megatag1 = LimelightHelpers.getBotPoseEstimate_wpiBlue(kSupplementaryCameraTable);
      var megatag2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(kSupplementaryCameraTable);

      if (megatag1.tagCount > 0) {
        inputs.supplementaryPose = PoseObservation.fromLimelight(megatag1);
      } else if (megatag2.tagCount > 0) {
        inputs.supplementaryPose = PoseObservation.fromLimelight(megatag2);
      } else {
        inputs.supplementaryPose = null;
      }
    } else {
      inputs.supplementaryPose = null;
    }

    cachedInputs.set(inputs);
    updateRobotOrientation();
  }
}
