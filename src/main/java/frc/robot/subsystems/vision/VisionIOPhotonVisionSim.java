// Vision simulation logic adapted from Team 254's 2025 codebase
// Copyright (c) 2025 Team 254
// Licensed under the MIT License
// https://github.com/Team254/FRC-2025-Public

package frc.robot.subsystems.vision;

import static frc.robot.subsystems.vision.VisionConstants.aprilTagLayout;
import static frc.robot.subsystems.vision.VisionConstants.kLimelightFourCameraPose;
import static frc.robot.subsystems.vision.VisionConstants.kLimelightFourTable;
import static frc.robot.subsystems.vision.VisionConstants.kLimelightThreeCameraPose;
import static frc.robot.subsystems.vision.VisionConstants.kLimelightThreeTable;
import static frc.robot.subsystems.vision.VisionConstants.kLimelightTwoCameraPose;
import static frc.robot.subsystems.vision.VisionConstants.kLimelightTwoTable;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.simulation.SimulatedRobotState;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Optional;
import org.photonvision.PhotonCamera;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;

/** IO implementation for physics sim using PhotonVision simulator. */
public class VisionIOPhotonVisionSim extends VisionIOLimelight {
  private static VisionSystemSim visionSim;
  private static boolean visionSimInitialized = false;
  private static boolean camerasInitialized = false;
  private static PhotonCamera leftCamera;
  private static PhotonCamera rightCamera;
  private static PhotonCamera rearCamera;
  private static PhotonCameraSim leftCameraSim;
  private static PhotonCameraSim rightCameraSim;
  private static PhotonCameraSim rearCameraSim;
  private static Transform3d leftCameraOffset;
  private static Transform3d rightCameraOffset;
  private static Transform3d rearCameraOffset;

  private final SimulatedRobotState simulatedRobotState;

  public VisionIOPhotonVisionSim() {
    super();
    simulatedRobotState = SimulatedRobotState.getInstance();

    if (!camerasInitialized) {
      leftCamera = new PhotonCamera(kLimelightFourTable);
      rightCamera = new PhotonCamera(kLimelightThreeTable);
      rearCamera = new PhotonCamera(kLimelightTwoTable);

      SimCameraProperties cameraProps = new SimCameraProperties();
      cameraProps.setCalibration(1280, 800, Rotation2d.fromDegrees(82));
      cameraProps.setFPS(60);
      cameraProps.setAvgLatencyMs(20);
      cameraProps.setLatencyStdDevMs(5);

      leftCameraSim = new PhotonCameraSim(leftCamera, cameraProps);
      rightCameraSim = new PhotonCameraSim(rightCamera, cameraProps);
      rearCameraSim = new PhotonCameraSim(rearCamera, cameraProps);

      leftCameraOffset =
          new Transform3d(
              kLimelightFourCameraPose[0],
              kLimelightFourCameraPose[1],
              kLimelightFourCameraPose[2],
              new Rotation3d(
                  Units.degreesToRadians(kLimelightFourCameraPose[3]),
                  Units.degreesToRadians(kLimelightFourCameraPose[4]),
                  Units.degreesToRadians(kLimelightFourCameraPose[5])));

      rightCameraOffset =
          new Transform3d(
              kLimelightThreeCameraPose[0],
              kLimelightThreeCameraPose[1],
              kLimelightThreeCameraPose[2],
              new Rotation3d(
                  Units.degreesToRadians(kLimelightThreeCameraPose[3]),
                  Units.degreesToRadians(kLimelightThreeCameraPose[4]),
                  Units.degreesToRadians(kLimelightThreeCameraPose[5])));

      rearCameraOffset =
          new Transform3d(
              kLimelightTwoCameraPose[0],
              kLimelightTwoCameraPose[1],
              kLimelightTwoCameraPose[2],
              new Rotation3d(
                  Units.degreesToRadians(kLimelightTwoCameraPose[3]),
                  Units.degreesToRadians(kLimelightTwoCameraPose[4]),
                  Units.degreesToRadians(kLimelightTwoCameraPose[5])));

      leftCameraSim.enableProcessedStream(true);
      leftCameraSim.enableRawStream(true);
      rightCameraSim.enableProcessedStream(true);
      rightCameraSim.enableRawStream(true);
      rearCameraSim.enableProcessedStream(true);
      rearCameraSim.enableRawStream(true);

      leftCameraSim.enableDrawWireframe(true);
      rightCameraSim.enableDrawWireframe(true);
      rearCameraSim.enableDrawWireframe(true);

      if (!visionSimInitialized) {
        visionSim = new VisionSystemSim("main");
        visionSim.addAprilTags(aprilTagLayout);
        visionSimInitialized = true;
      }

      visionSim.addCamera(leftCameraSim, leftCameraOffset);
      visionSim.addCamera(rightCameraSim, rightCameraOffset);
      visionSim.addCamera(rearCameraSim, rearCameraOffset);
      camerasInitialized = true;
    }
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    // Avoid feedback loops by using simulated robot state instead of RobotState
    Pose2d robotPose = simulatedRobotState.getLatestFieldToRobot();
    if (robotPose == null) {
      robotPose = Pose2d.kZero;
    }

    visionSim.update(robotPose);

    NetworkTable leftTable = NetworkTableInstance.getDefault().getTable(kLimelightFourTable);
    NetworkTable rightTable = NetworkTableInstance.getDefault().getTable(kLimelightThreeTable);
    NetworkTable rearTable = NetworkTableInstance.getDefault().getTable(kLimelightTwoTable);

    writeToTable(leftCamera.getAllUnreadResults(), leftTable, leftCameraOffset);
    writeToTable(rightCamera.getAllUnreadResults(), rightTable, rightCameraOffset);
    writeToTable(rearCamera.getAllUnreadResults(), rearTable, rearCameraOffset);

    super.updateInputs(inputs);
  }

  /**
   * Converts field-to-camera to field-to-robot using the camera offset (robot-to-camera). Limelight
   * botpose is the robot pose, not the camera pose.
   */
  private List<Double> getBotpose(
      Transform3d fieldToCamera,
      Transform3d robotToCamera,
      int numTags,
      PhotonPipelineResult result) {
    if (result == null || result.targets.isEmpty()) {
      return null;
    }

    Pose3d fieldToCameraPose3d = new Pose3d().plus(fieldToCamera);
    Pose3d fieldToRobotPose3d = fieldToCameraPose3d.plus(robotToCamera.inverse());
    Pose2d fieldToRobot2d =
        new Pose2d(
            fieldToRobotPose3d.getTranslation().toTranslation2d(),
            fieldToRobotPose3d.getRotation().toRotation2d());

    List<Double> poseData =
        new ArrayList<>(
            Arrays.asList(
                fieldToRobot2d.getX(),
                fieldToRobot2d.getY(),
                0.0,
                0.0,
                0.0,
                Units.radiansToDegrees(fieldToRobot2d.getRotation().getRadians()),
                result.metadata.getLatencyMillis() / 1000.0,
                (double) numTags,
                0.0,
                0.0,
                result.getBestTarget().getArea()));

    for (var target : result.targets) {
      poseData.addAll(
          Arrays.asList(
              (double) target.getFiducialId(),
              target.getYaw(), // txnc
              target.getPitch(), // tync
              target.getArea(), // ta
              0.0, // distToCamera
              0.0, // distToRobot
              target.getPoseAmbiguity() // ambiguity
              ));
    }
    return poseData;
  }

  private void writeToTable(
      List<PhotonPipelineResult> results, NetworkTable table, Transform3d robotToCamera) {
    boolean seesTarget = false;
    for (var result : results) {
      List<Double> poseData = null;
      if (result.getMultiTagResult().isPresent()) {
        var multiTagResult = result.getMultiTagResult().get();
        Transform3d fieldToCamera = multiTagResult.estimatedPose.best;
        poseData =
            getBotpose(fieldToCamera, robotToCamera, multiTagResult.fiducialIDsUsed.size(), result);
      } else if (result.hasTargets()) {
        var bestTarget = result.getBestTarget();
        Optional<Pose3d> tagPose = aprilTagLayout.getTagPose(bestTarget.getFiducialId());
        if (tagPose.isPresent()) {
          Transform3d fieldToCamera =
              new Transform3d(
                      tagPose.get().getTranslation().minus(new Translation3d()),
                      tagPose.get().getRotation())
                  .plus(bestTarget.bestCameraToTarget.inverse());
          poseData = getBotpose(fieldToCamera, robotToCamera, 1, result);
        }
      }

      if (poseData != null) {
        table
            .getEntry("botpose_wpiblue")
            .setDoubleArray(poseData.stream().mapToDouble(Double::doubleValue).toArray());
        table
            .getEntry("botpose_orb_wpiblue")
            .setDoubleArray(poseData.stream().mapToDouble(Double::doubleValue).toArray());
        table
            .getEntry("stddevs")
            .setDoubleArray(
                new double[] {0.3, 0.3, 0.0, 0.0, 0.0, 0.3, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
        seesTarget = true;
      }
      table.getEntry("cl").setDouble(result.metadata.getLatencyMillis() / 1000.0);
      table.getEntry("tl").setDouble(result.metadata.getLatencyMillis() / 1000.0);
    }
    if (!seesTarget) {
      table.getEntry("tl").setDouble(0.02);
    }
    table.getEntry("tv").setInteger(seesTarget ? 1 : 0);
  }
}
