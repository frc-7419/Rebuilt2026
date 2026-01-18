package frc.robot.subsystems.vision;

import static edu.wpi.first.units.Units.RadiansPerSecond;
import static frc.robot.subsystems.turret.TurretConstants.kTurretOffset;
import static frc.robot.subsystems.vision.VisionConstants.kSupplementaryCameraPose;
import static frc.robot.subsystems.vision.VisionConstants.kSupplementaryCameraTable;
import static frc.robot.subsystems.vision.VisionConstants.kTurretCameraPose;
import static frc.robot.subsystems.vision.VisionConstants.kTurretCameraTable;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.RobotState;
import frc.robot.util.LimelightHelpers;
import java.util.concurrent.atomic.AtomicReference;

public class VisionIOLimelight implements VisionIO {
  private final NetworkTable turretTable;
  private final NetworkTable supplementaryTable;
  private final RobotState robotState;
  private final AtomicReference<VisionIOInputs> cachedInputs =
      new AtomicReference<>(new VisionIOInputs());

  public VisionIOLimelight() {
    turretTable = NetworkTableInstance.getDefault().getTable(kTurretCameraTable);
    supplementaryTable = NetworkTableInstance.getDefault().getTable(kSupplementaryCameraTable);
    robotState = RobotState.getInstance();
    configureLimelightSettings();
  }

  private void configureLimelightSettings() {
    updateTurretCameraPose();
    supplementaryTable
        .getEntry("camerapose_robotspace_set")
        .setDoubleArray(kSupplementaryCameraPose);
  }

  private void updateTurretCameraPose() {
    var latestTurretRotation = robotState.getLatestRobotToTurret();
    if (latestTurretRotation != null) {

      Transform3d robotToTurret =
          new Transform3d(
              new Translation3d(
                  kTurretOffset.getTranslation().getX(),
                  kTurretOffset.getTranslation().getY(),
                  0.0),
              new Rotation3d(0.0, 0.0, latestTurretRotation.getValue().getRadians()));

      Transform3d turretToCamera =
          new Transform3d(
              new Translation3d(kTurretCameraPose[0], kTurretCameraPose[1], kTurretCameraPose[2]),
              new Rotation3d(
                  Units.degreesToRadians(kTurretCameraPose[3]),
                  Units.degreesToRadians(kTurretCameraPose[4]),
                  Units.degreesToRadians(kTurretCameraPose[5])));

      Transform3d robotToCamera = robotToTurret.plus(turretToCamera);

      double[] cameraPoseArray = {
        robotToCamera.getX(),
        robotToCamera.getY(),
        robotToCamera.getZ(),
        Units.radiansToDegrees(robotToCamera.getRotation().getX()),
        Units.radiansToDegrees(robotToCamera.getRotation().getY()),
        Units.radiansToDegrees(robotToCamera.getRotation().getZ())
      };

      turretTable.getEntry("camerapose_robotspace_set").setDoubleArray(cameraPoseArray);
    } else {
      turretTable.getEntry("camerapose_robotspace_set").setDoubleArray(kTurretCameraPose);
    }
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
      double combinedYawRateRadPerS =
          robotSpeeds.omegaRadiansPerSecond + turretAngularVelocityRadPerS;
      double combinedYawRateDegPerS = Units.radiansToDegrees(combinedYawRateRadPerS);

      LimelightHelpers.SetRobotOrientation(
          kTurretCameraTable,
          fieldToTurretRotation.getDegrees(),
          combinedYawRateDegPerS,
          0.0,
          0.0,
          0.0,
          0.0);

      Rotation2d robotRotation = latestRobotPose.getValue().getRotation();
      double robotYawRateDegPerS = Units.radiansToDegrees(robotSpeeds.omegaRadiansPerSecond);

      LimelightHelpers.SetRobotOrientation(
          kSupplementaryCameraTable,
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

    boolean turretSeesTarget = turretTable.getEntry("tv").getDouble(0) == 1.0;
    boolean supplementarySeesTarget = supplementaryTable.getEntry("tv").getDouble(0) == 1.0;

    inputs.turretHasTarget = turretSeesTarget;
    inputs.supplementaryHasTarget = supplementarySeesTarget;

    if (turretSeesTarget) {
      var megatag2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(kTurretCameraTable);
      var megatag1 = LimelightHelpers.getBotPoseEstimate_wpiBlue(kTurretCameraTable);

      if (megatag2 != null && megatag2.tagCount > 0) {
        inputs.turretPose = PoseObservation.fromLimelight(megatag2);
      } else if (megatag1 != null && megatag1.tagCount > 0) {
        inputs.turretPose = PoseObservation.fromLimelight(megatag1);
      } else {
        inputs.turretPose = null;
      }
    } else {
      inputs.turretPose = null;
    }

    if (supplementarySeesTarget) {
      var megatag2 =
          LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(kSupplementaryCameraTable);
      var megatag1 = LimelightHelpers.getBotPoseEstimate_wpiBlue(kSupplementaryCameraTable);

      if (megatag2 != null && megatag2.tagCount > 0) {
        inputs.supplementaryPose = PoseObservation.fromLimelight(megatag2);
      } else if (megatag1 != null && megatag1.tagCount > 0) {
        inputs.supplementaryPose = PoseObservation.fromLimelight(megatag1);
      } else {
        inputs.supplementaryPose = null;
      }
    } else {
      inputs.supplementaryPose = null;
    }

    cachedInputs.set(inputs);
    updateRobotOrientation();
    updateTurretCameraPose();
  }
}
