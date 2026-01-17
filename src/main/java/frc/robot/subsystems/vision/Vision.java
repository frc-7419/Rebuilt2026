// Vision processing logic adapted from Team 254's 2024 codebase
// Copyright (c) 2024 Team 254
// Licensed under the MIT License
// https://github.com/Team254/FRC-2024-Public

package frc.robot.subsystems.vision;

import static edu.wpi.first.units.Units.RadiansPerSecond;
import static frc.robot.subsystems.vision.VisionConstants.kSupplementaryCameraPose;
import static frc.robot.subsystems.vision.VisionConstants.kTurretCameraPose;
import static frc.robot.subsystems.vision.VisionConstants.kUseMegatag1ForHubTagsOnTurret;

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
import java.util.Arrays;
import java.util.HashSet;
import java.util.List;
import java.util.Optional;
import java.util.Set;
import java.util.stream.Collectors;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Vision extends SubsystemBase {
  private final VisionIO io;
  private final RobotState robotState;
  private final VisionIO.VisionIOInputs inputs = new VisionIO.VisionIOInputs();
  private Drive drive;

  private double lastProcessedTurretTimestamp = 0.0;
  private double lastProcessedSupplementaryTimestamp = 0.0;

  private static final Set<Integer> kHubTagsBlue = new HashSet<>(List.of(26, 25));
  private static final Set<Integer> kHubTagsRed = new HashSet<>(List.of(9, 10));

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
    Logger.recordOutput("Vision/turretHasTarget", inputs.turretHasTarget);
    Logger.recordOutput("Vision/supplementaryHasTarget", inputs.supplementaryHasTarget);

    // Log camera poses in field space
    logCameraPoses();

    if (inputs.turretHasTarget && inputs.turretPose != null) {
      updateVision(inputs.turretPose, true);
    }
    if (inputs.supplementaryHasTarget && inputs.supplementaryPose != null) {
      updateVision(inputs.supplementaryPose, false);
    }

    Logger.recordOutput("Vision/latencyPeriodicSec", Timer.getFPGATimestamp() - timestamp);
  }

  @AutoLogOutput(key = "Vision/TurretCameraPose")
  private Pose3d turretCameraPose = new Pose3d();

  @AutoLogOutput(key = "Vision/SupplementaryCameraPose")
  private Pose3d supplementaryCameraPose = new Pose3d();

  private void logCameraPoses() {
    var latestRobotPose = robotState.getLatestFieldToRobot();
    if (latestRobotPose == null) {
      turretCameraPose = new Pose3d();
      supplementaryCameraPose = new Pose3d();
      return;
    }

    Pose2d robotPose = latestRobotPose.getValue();
    Pose3d robotPose3d = new Pose3d(robotPose);

    // Calculate turret camera pose
    var latestTurretRotation = robotState.getLatestRobotToTurret();
    if (latestTurretRotation != null) {
      Transform3d robotToTurret3d =
          new Transform3d(
              new Translation3d(),
              new Rotation3d(0.0, 0.0, latestTurretRotation.getValue().getRadians()));
      Transform3d turretToCamera3d = getTurretToCameraTransform(true);
      Transform3d robotToTurretCamera3d = robotToTurret3d.plus(turretToCamera3d);
      turretCameraPose = robotPose3d.transformBy(robotToTurretCamera3d);
    } else {
      turretCameraPose = new Pose3d();
    }

    // Calculate supplementary camera pose (fixed to robot, not turret)
    Transform3d robotToSupplementaryCamera3d = getTurretToCameraTransform(false);
    supplementaryCameraPose = robotPose3d.transformBy(robotToSupplementaryCamera3d);
  }

  private void updateVision(PoseObservation observation, boolean isTurretCamera) {
    String logPrefix = "Vision/" + (isTurretCamera ? "Turret/" : "Supplementary/");
    double timestamp = observation.timestampSeconds;
    double lastProcessedTimestamp =
        isTurretCamera ? lastProcessedTurretTimestamp : lastProcessedSupplementaryTimestamp;

    if (timestamp == lastProcessedTimestamp) {
      return;
    }

    Optional<RobotState.VisionObservation> megatag2Estimate =
        processMegatag2Estimate(observation, isTurretCamera, logPrefix);
    Optional<RobotState.VisionObservation> megatag1Estimate =
        processMegatag1Estimate(observation, isTurretCamera);

    boolean shouldPrioritizeMegatag1 =
        kUseMegatag1ForHubTagsOnTurret
            && isTurretCamera
            && shouldUseMegatag1(observation, isTurretCamera, logPrefix);

    boolean usedMegatag1 = false;
    boolean usedMegatag2 = false;

    if (shouldPrioritizeMegatag1 && megatag1Estimate.isPresent()) {
      Logger.recordOutput(logPrefix + "Megatag1Estimate", megatag1Estimate.get().visionPose());
      addVisionObservation(megatag1Estimate.get());
      usedMegatag1 = true;
    } else {
      if (megatag2Estimate.isPresent()) {
        if (shouldUseMegatag2(observation, isTurretCamera, logPrefix)) {
          Logger.recordOutput(logPrefix + "Megatag2Estimate", megatag2Estimate.get().visionPose());
          addVisionObservation(megatag2Estimate.get());
          usedMegatag2 = true;
        } else {
          Logger.recordOutput(
              logPrefix + "Megatag2EstimateRejected", megatag2Estimate.get().visionPose());
        }
      }

      if (!usedMegatag2 && megatag1Estimate.isPresent()) {
        if (shouldUseMegatag1(observation, isTurretCamera, logPrefix)) {
          Logger.recordOutput(logPrefix + "Megatag1Estimate", megatag1Estimate.get().visionPose());
          addVisionObservation(megatag1Estimate.get());
          usedMegatag1 = true;
        } else {
          Logger.recordOutput(
              logPrefix + "Megatag1EstimateRejected", megatag1Estimate.get().visionPose());
        }
      }
    }

    if (isTurretCamera) {
      lastProcessedTurretTimestamp = timestamp;
    } else {
      lastProcessedSupplementaryTimestamp = timestamp;
    }
  }

  private boolean shouldUseMegatag1(
      PoseObservation observation, boolean isTurretCamera, String logPrefix) {
    final int kExpectedTagCount = 2;

    if (observation.tagCount < kExpectedTagCount) {
      Logger.recordOutput(logPrefix + "tagCount", false);
      return false;
    }
    Logger.recordOutput(logPrefix + "tagCount", true);

    if (observation.fiducialIds == null || observation.fiducialIds.length < 1) {
      Logger.recordOutput(logPrefix + "fiducialIdsEmpty", false);
      return false;
    }
    Logger.recordOutput(logPrefix + "fiducialIdsEmpty", true);

    if (observation.estimatedPose.getTranslation().getNorm() < 1.0) {
      Logger.recordOutput(logPrefix + "poseNorm", false);
      return false;
    }
    Logger.recordOutput(logPrefix + "poseNorm", true);

    Set<Integer> seenTagIds =
        Arrays.stream(observation.fiducialIds)
            .boxed()
            .collect(Collectors.toCollection(HashSet::new));
    Set<Integer> expectedHubTags = robotState.isRedAlliance() ? kHubTagsRed : kHubTagsBlue;
    boolean matchesHubTags = expectedHubTags.equals(seenTagIds);
    Logger.recordOutput(logPrefix + "hubTagsMatch", matchesHubTags);
    return matchesHubTags;
  }

  private boolean shouldUseMegatag2(
      PoseObservation observation, boolean isTurretCamera, String logPrefix) {
    return isMotionAcceptable(observation.timestampSeconds, isTurretCamera, logPrefix);
  }

  private boolean isMotionAcceptable(double timestamp, boolean isTurretCamera, String logPrefix) {
    final double kMaxYawRateRadPerS = Units.degreesToRadians(100.0);

    var robotSpeeds = robotState.getLatestRobotRelativeChassisSpeed();
    double robotYawRateRadPerS = Math.abs(robotSpeeds.omegaRadiansPerSecond);

    if (isTurretCamera) {
      var turretAngularVelocity = robotState.getLatestTurretAngularVelocity();
      double turretYawRateRadPerS = Math.abs(turretAngularVelocity.in(RadiansPerSecond));
      double combinedYawRateRadPerS = robotYawRateRadPerS + turretYawRateRadPerS;

      if (combinedYawRateRadPerS > kMaxYawRateRadPerS) {
        Logger.recordOutput(logPrefix + "motionAcceptable", false);
        return false;
      }
      Logger.recordOutput(logPrefix + "motionAcceptable", true);
    } else {
      if (robotYawRateRadPerS > kMaxYawRateRadPerS) {
        Logger.recordOutput(logPrefix + "motionAcceptable", false);
        return false;
      }
      Logger.recordOutput(logPrefix + "motionAcceptable", true);
    }

    return true;
  }

  private Optional<Pose2d> getFieldToRobotEstimate(
      PoseObservation observation, boolean isTurretCamera) {
    Pose2d fieldToCamera = observation.estimatedPose;
    if (fieldToCamera.getX() == 0.0) {
      return Optional.empty();
    }

    Optional<Rotation2d> robotToTurret = robotState.getRobotToTurret(observation.timestampSeconds);
    if (robotToTurret.isEmpty()) {
      return Optional.empty();
    }

    Transform3d turretToCamera3d = getTurretToCameraTransform(isTurretCamera);
    Transform3d cameraToTurret3d = turretToCamera3d.inverse();
    Transform2d cameraToTurret2d =
        new Transform2d(
            new Translation2d(cameraToTurret3d.getX(), cameraToTurret3d.getY()),
            new Rotation2d(cameraToTurret3d.getRotation().getZ()));
    Pose2d fieldToTurret = fieldToCamera.transformBy(cameraToTurret2d);

    if (isTurretCamera) {
      Transform2d turretToRobot =
          new Transform2d(new Translation2d(), robotToTurret.get().unaryMinus());
      return Optional.of(fieldToTurret.transformBy(turretToRobot));
    } else {
      return Optional.of(fieldToTurret);
    }
  }

  private Transform3d getTurretToCameraTransform(boolean isTurretCamera) {
    if (isTurretCamera) {
      // kTurretCameraPose: [forward, side, up, roll(deg), pitch(deg), yaw(deg)]
      // Convert rotation angles from degrees to radians
      return new Transform3d(
          new Translation3d(kTurretCameraPose[0], kTurretCameraPose[1], kTurretCameraPose[2]),
          new Rotation3d(
              Units.degreesToRadians(kTurretCameraPose[3]),
              Units.degreesToRadians(kTurretCameraPose[4]),
              Units.degreesToRadians(kTurretCameraPose[5])));
    } else {
      // kSupplementaryCameraPose: [forward, side, up, roll(deg), pitch(deg),
      // yaw(deg)]
      return new Transform3d(
          new Translation3d(
              kSupplementaryCameraPose[0],
              kSupplementaryCameraPose[1],
              kSupplementaryCameraPose[2]),
          new Rotation3d(
              Units.degreesToRadians(kSupplementaryCameraPose[3]),
              Units.degreesToRadians(kSupplementaryCameraPose[4]),
              Units.degreesToRadians(kSupplementaryCameraPose[5])));
    }
  }

  private Optional<RobotState.VisionObservation> processMegatag2Estimate(
      PoseObservation observation, boolean isTurretCamera, String logPrefix) {
    Optional<Pose2d> loggedFieldToRobot = robotState.getFieldToRobot(observation.timestampSeconds);
    if (loggedFieldToRobot.isEmpty()) {
      return Optional.empty();
    }

    Optional<Pose2d> fieldToRobotEstimate = getFieldToRobotEstimate(observation, isTurretCamera);
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
      Set<Integer> hubTags = new HashSet<>(robotState.isRedAlliance() ? kHubTagsRed : kHubTagsBlue);
      hubTags.removeAll(
          Arrays.stream(observation.fiducialIds)
              .boxed()
              .collect(Collectors.toCollection(HashSet::new)));
      boolean seesHubTags = hubTags.size() < 2;

      if (observation.fiducialIds.length >= 2 && observation.tagCount > 0) {
        xyStdDevMeters = 0.2;
      } else if (seesHubTags && observation.tagCount > 0) {
        xyStdDevMeters = 0.5;
      } else if (observation.tagCount > 0 && poseDifferenceMeters < 0.5) {
        xyStdDevMeters = 0.5;
      } else if (observation.tagCount > 0 && poseDifferenceMeters < 0.3) {
        xyStdDevMeters = 1.0;
      } else if (observation.fiducialIds.length > 1) {
        xyStdDevMeters = 1.2;
      } else {
        xyStdDevMeters = 2.0;
      }

      Logger.recordOutput(logPrefix + "megatag2StdDevMeters", xyStdDevMeters);
      Logger.recordOutput(logPrefix + "megatag2PoseDifferenceMeters", poseDifferenceMeters);

      Matrix<N3, N1> stdDevs =
          VecBuilder.fill(xyStdDevMeters, xyStdDevMeters, Units.degreesToRadians(50.0));
      Pose2d correctedPose =
          new Pose2d(
              fieldToRobotEstimate.get().getTranslation(), loggedFieldToRobot.get().getRotation());
      return Optional.of(
          new RobotState.VisionObservation(observation.timestampSeconds, correctedPose, stdDevs));
    }
    return Optional.empty();
  }

  private Optional<RobotState.VisionObservation> processMegatag1Estimate(
      PoseObservation observation, boolean isTurretCamera) {
    Optional<Pose2d> loggedFieldToRobot = robotState.getFieldToRobot(observation.timestampSeconds);
    if (loggedFieldToRobot.isEmpty()) {
      return Optional.empty();
    }

    Optional<Pose2d> fieldToRobotEstimate = getFieldToRobotEstimate(observation, isTurretCamera);
    if (fieldToRobotEstimate.isEmpty()) {
      return Optional.empty();
    }

    double poseDifferenceMeters =
        fieldToRobotEstimate
            .get()
            .getTranslation()
            .getDistance(loggedFieldToRobot.get().getTranslation());

    if (observation.fiducialIds != null && observation.fiducialIds.length > 0) {
      double xyStdDevMeters = 1.0;
      double rotationStdDevDeg = 12.0;

      if (observation.fiducialIds.length >= 2) {
        xyStdDevMeters = 0.5;
        rotationStdDevDeg = 6.0;
      } else if (observation.tagCount > 0 && poseDifferenceMeters < 0.5) {
        xyStdDevMeters = 1.0;
        rotationStdDevDeg = 12.0;
      } else if (observation.tagCount > 0 && poseDifferenceMeters < 0.3) {
        xyStdDevMeters = 2.0;
        rotationStdDevDeg = 30.0;
      }

      Matrix<N3, N1> stdDevs =
          VecBuilder.fill(
              xyStdDevMeters, xyStdDevMeters, Units.degreesToRadians(rotationStdDevDeg));
      return Optional.of(
          new RobotState.VisionObservation(
              observation.timestampSeconds, fieldToRobotEstimate.get(), stdDevs));
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
