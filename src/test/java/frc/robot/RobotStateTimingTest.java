package frc.robot;

import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import org.junit.jupiter.api.Test;

class RobotStateTimingTest {
  @Test
  void visionCorrectionKeepsSampleTimeAndResetClearsOldHistoryImmediately() {
    RobotState state = RobotState.getInstance();
    state.resetPose(Pose2d.kZero, 1);
    long sequence = state.getPoseResetSequence();
    state.addOdometryMeasurement(1.004, new Pose2d(1, 0, Rotation2d.kZero));
    state.addOdometryMeasurement(1.008, new Pose2d(2, 0, Rotation2d.kZero));
    Pose2d correction = new Pose2d(2.1, .1, Rotation2d.fromDegrees(20));
    state.correctLatestPose(correction);
    assertEquals(1.008, state.getLatestFieldToRobot().getKey());
    assertEquals(correction, state.getLatestFieldToRobotPose());
    assertEquals(correction, state.getEstimatedPose());
    assertEquals(1.5, state.getFieldToRobot(1.006).orElseThrow().getX(), .1);
    assertEquals(sequence, state.getPoseResetSequence());
    state.resetPose(Pose2d.kZero, 2);
    assertEquals(sequence + 1, state.getPoseResetSequence());
    assertEquals(1, state.getPoseBuffer().getInternalBuffer().size());
    assertEquals(2, state.getLatestFieldToRobot().getKey());
  }
}
