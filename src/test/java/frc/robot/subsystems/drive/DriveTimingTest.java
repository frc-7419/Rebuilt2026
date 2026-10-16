package frc.robot.subsystems.drive;

import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.RobotState;
import org.junit.jupiter.api.Test;

class DriveTimingTest {
  private static class Samples implements ModuleIO {
    double[] times = {1, 1.004};
    double[] positions = {0, 1};

    @Override
    public void updateInputs(ModuleIOInputs inputs) {
      inputs.driveConnected = true;
      inputs.turnConnected = true;
      inputs.turnEncoderConnected = true;
      inputs.odometryTimestamps = times;
      inputs.odometryDrivePositionsRad = positions;
      inputs.odometryTurnPositions = new Rotation2d[positions.length];
      java.util.Arrays.fill(inputs.odometryTurnPositions, Rotation2d.kZero);
      inputs.drivePositionRad = positions.length == 0 ? 1 : positions[positions.length - 1];
    }
  }

  @Test
  void noSamplesDoNotRefreshPoseAndFusionAndResetPublishImmediately() {
    assertTrue(HAL.initialize(500, 0));
    RobotState.getInstance().resetPose(Pose2d.kZero, 0);
    var io = new Samples();
    var drive = new Drive(new GyroIO() {}, io, io, io, io);
    var state = RobotState.getInstance();
    drive.periodic();
    assertEquals(1.004, state.getLatestFieldToRobot().getKey());
    assertTrue(state.getPoseBuffer().getInternalBuffer().containsKey(1.0));
    Pose2d beforeVision = state.getEstimatedPose();
    drive.addVisionMeasurement(
        new Pose2d(beforeVision.getX() + .5, 0, Rotation2d.kZero),
        1.004,
        VecBuilder.fill(.001, .001, 1e6));
    assertEquals(drive.getPose(), state.getEstimatedPose());
    assertTrue(state.getEstimatedPose().getX() > beforeVision.getX());
    assertEquals(1.004, state.getLatestFieldToRobot().getKey());
    io.times = new double[0];
    io.positions = new double[0];
    drive.periodic();
    assertEquals(1.004, state.getLatestFieldToRobot().getKey());
    long resets = state.getPoseResetSequence();
    Pose2d reset = new Pose2d(2, 3, Rotation2d.fromDegrees(90));
    drive.setPose(reset);
    assertEquals(reset, state.getLatestFieldToRobotPose());
    assertEquals(1.004, state.getLatestFieldToRobot().getKey());
    assertEquals(resets + 1, state.getPoseResetSequence());
    // A malformed module batch is dropped, rather than indexing beyond the available positions.
    io.times = new double[] {1.008};
    drive.periodic();
    assertEquals(1.004, state.getLatestFieldToRobot().getKey());
  }
}
