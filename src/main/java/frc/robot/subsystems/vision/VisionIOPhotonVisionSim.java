// Vision simulation logic adapted from Team 254's 2025 codebase
// Copyright (c) 2025 Team 254
// Licensed under the MIT License
// https://github.com/Team254/FRC-2025-Public

package frc.robot.subsystems.vision;

import static frc.robot.subsystems.vision.VisionConstants.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.RobotState;
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
  private static PhotonCamera turretCamera;
  private static PhotonCamera supplementaryCamera;
  private static PhotonCameraSim turretCameraSim;
  private static PhotonCameraSim supplementaryCameraSim;
  private static Transform3d baseTurretToCamera; // Base transform from turret center to camera

  private final RobotState robotState;
  private final SimulatedRobotState simulatedRobotState;

  public VisionIOPhotonVisionSim() {
    super();
    robotState = RobotState.getInstance();
    simulatedRobotState = SimulatedRobotState.getInstance();

    if (!camerasInitialized) {
      turretCamera = new PhotonCamera(kTurretCameraTable);
      supplementaryCamera = new PhotonCamera(kSupplementaryCameraTable);

      SimCameraProperties cameraProps = new SimCameraProperties();
      cameraProps.setCalibration(1280, 800, Rotation2d.fromDegrees(82));
      cameraProps.setFPS(60);
      cameraProps.setAvgLatencyMs(20);
      cameraProps.setLatencyStdDevMs(5);

      turretCameraSim = new PhotonCameraSim(turretCamera, cameraProps);
      supplementaryCameraSim = new PhotonCameraSim(supplementaryCamera, cameraProps);

      baseTurretToCamera =
          new Transform3d(
              kTurretCameraPose[0],
              kTurretCameraPose[1],
              kTurretCameraPose[2],
              new Rotation3d(
                  Units.degreesToRadians(kTurretCameraPose[3]),
                  Units.degreesToRadians(kTurretCameraPose[4]),
                  Units.degreesToRadians(kTurretCameraPose[5])));

      Transform3d supplementaryCameraOffset =
          new Transform3d(
              kSupplementaryCameraPose[0],
              kSupplementaryCameraPose[1],
              kSupplementaryCameraPose[2],
              new Rotation3d(
                  Units.degreesToRadians(kSupplementaryCameraPose[3]),
                  Units.degreesToRadians(kSupplementaryCameraPose[4]),
                  Units.degreesToRadians(kSupplementaryCameraPose[5])));
      turretCameraSim.enableProcessedStream(true);
      turretCameraSim.enableRawStream(true);
      supplementaryCameraSim.enableProcessedStream(true);
      supplementaryCameraSim.enableRawStream(true);

      turretCameraSim.enableDrawWireframe(true);
      supplementaryCameraSim.enableDrawWireframe(true);

      if (!visionSimInitialized) {
        visionSim = new VisionSystemSim("main");
        visionSim.addAprilTags(aprilTagLayout);
        visionSimInitialized = true;
      }

      Transform3d initialRobotToTurret = new Transform3d(new Translation3d(), new Rotation3d());
      Transform3d initialRobotToCamera = initialRobotToTurret.plus(baseTurretToCamera);
      visionSim.addCamera(turretCameraSim, initialRobotToCamera);
      visionSim.addCamera(supplementaryCameraSim, supplementaryCameraOffset);
      camerasInitialized = true;
    }
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    // Avioid feedback loops by using simulated robot state instead of RobotState
    Pose2d robotPose = simulatedRobotState.getLatestFieldToRobot();
    if (robotPose == null) {
      robotPose = Pose2d.kZero;
    }

    var latestTurretRotation = robotState.getLatestRobotToTurret();
    if (latestTurretRotation != null && baseTurretToCamera != null) {
      Transform3d robotToTurret =
          new Transform3d(
              new Translation3d(),
              new Rotation3d(0.0, 0.0, latestTurretRotation.getValue().getRadians()));

      Rotation3d turretRot = new Rotation3d(0.0, 0.0, latestTurretRotation.getValue().getRadians());
      Transform3d rotatedTurretToCamera =
          new Transform3d(
              baseTurretToCamera.getTranslation().rotateBy(turretRot),
              baseTurretToCamera.getRotation().rotateBy(turretRot));

      Transform3d dynamicRobotToCamera = robotToTurret.plus(rotatedTurretToCamera);

      visionSim.adjustCamera(turretCameraSim, dynamicRobotToCamera);
    }

    visionSim.update(robotPose);

    NetworkTable turretTable = NetworkTableInstance.getDefault().getTable(kTurretCameraTable);
    NetworkTable supplementaryTable =
        NetworkTableInstance.getDefault().getTable(kSupplementaryCameraTable);

    writeToTable(turretCamera.getAllUnreadResults(), turretTable, turretCameraSim, true);
    writeToTable(
        supplementaryCamera.getAllUnreadResults(),
        supplementaryTable,
        supplementaryCameraSim,
        false);

    super.updateInputs(inputs);
  }

  private List<Double> getBotpose(
      Transform3d fieldToCamera,
      int numTags,
      PhotonPipelineResult result,
      PhotonCameraSim cameraSim,
      boolean isTurretCamera) {
    if (result == null || result.targets.isEmpty()) {
      return null;
    }

    Pose3d fieldToCameraPose3d = new Pose3d().plus(fieldToCamera);
    Pose2d fieldToCamera2d =
        new Pose2d(
            fieldToCameraPose3d.getTranslation().toTranslation2d(),
            fieldToCameraPose3d.getRotation().toRotation2d());

    List<Double> poseData =
        new ArrayList<>(
            Arrays.asList(
                fieldToCamera2d.getX(),
                fieldToCamera2d.getY(),
                0.0,
                0.0,
                0.0,
                Units.radiansToDegrees(fieldToCamera2d.getRotation().getRadians()),
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
      List<PhotonPipelineResult> results,
      NetworkTable table,
      PhotonCameraSim cameraSim,
      boolean isTurretCamera) {
    boolean seesTarget = false;
    for (var result : results) {
      List<Double> poseData = null;
      if (result.getMultiTagResult().isPresent()) {
        var multiTagResult = result.getMultiTagResult().get();
        Transform3d best = multiTagResult.estimatedPose.best;
        poseData =
            getBotpose(
                best, multiTagResult.fiducialIDsUsed.size(), result, cameraSim, isTurretCamera);
      } else if (result.hasTargets()) {
        var bestTarget = result.getBestTarget();
        Optional<Pose3d> tagPose = aprilTagLayout.getTagPose(bestTarget.getFiducialId());
        if (tagPose.isPresent()) {
          Transform3d best =
              new Transform3d(
                      tagPose.get().getTranslation().minus(new Translation3d()),
                      tagPose.get().getRotation())
                  .plus(bestTarget.bestCameraToTarget.inverse());
          poseData = getBotpose(best, 1, result, cameraSim, isTurretCamera);
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
    }
    table.getEntry("tv").setInteger(seesTarget ? 1 : 0);
  }
}
