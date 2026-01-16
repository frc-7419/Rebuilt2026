package frc.robot.simulation;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.wpilibj.Timer;
import org.littletonrobotics.junction.AutoLogOutput;

/**
 * Simulated robot state that maintains ground truth pose from the drive subsystem. This is separate
 * from RobotState to avoid feedback loops in vision simulation.
 */
public class SimulatedRobotState {
  private static final double LOOKBACK_TIME_SEC = 2.0;

  private static SimulatedRobotState instance;

  public static SimulatedRobotState getInstance() {
    if (instance == null) instance = new SimulatedRobotState();
    return instance;
  }

  private final TimeInterpolatableBuffer<Pose2d> fieldToRobotSimulatedTruth =
      TimeInterpolatableBuffer.createBuffer(LOOKBACK_TIME_SEC);

  private SimulatedRobotState() {
    // Initialize with zero pose
    fieldToRobotSimulatedTruth.addSample(0.0, Pose2d.kZero);
  }

  /**
   * Adds a ground truth pose measurement from the drive subsystem. This should be called from
   * simulation periodic with the pose from Drive.getPose().
   *
   * @param pose The ground truth pose from odometry (not including vision corrections)
   */
  public synchronized void addFieldToRobot(Pose2d pose) {
    double timestamp = Timer.getFPGATimestamp();
    fieldToRobotSimulatedTruth.addSample(timestamp, pose);
  }

  /**
   * Gets the latest ground truth pose.
   *
   * @return The latest ground truth pose, or null if no pose is available
   */
  @AutoLogOutput
  public synchronized Pose2d getLatestFieldToRobot() {
    var entry = fieldToRobotSimulatedTruth.getInternalBuffer().lastEntry();
    if (entry == null) {
      return null;
    }
    return entry.getValue();
  }

  /**
   * Gets the ground truth pose at a specific timestamp.
   *
   * @param timestamp The timestamp to query
   * @return The pose at that timestamp, or empty if not available
   */
  public synchronized java.util.Optional<Pose2d> getFieldToRobot(double timestamp) {
    return fieldToRobotSimulatedTruth.getSample(timestamp);
  }

  /**
   * Resets the simulated robot state to a new pose.
   *
   * @param pose The new pose to reset to
   */
  public synchronized void resetPose(Pose2d pose) {
    fieldToRobotSimulatedTruth.clear();
    fieldToRobotSimulatedTruth.addSample(Timer.getFPGATimestamp(), pose);
  }
}
