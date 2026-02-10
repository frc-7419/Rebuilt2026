package frc.robot.subsystems.vision;

import org.littletonrobotics.junction.AutoLog;

public interface VisionIO {
  @AutoLog
  public static class VisionIOInputs {
    public boolean connected = false;
    public boolean limelightFourHasTarget;
    public boolean limelightThreeHasTarget;
    public PoseObservation limelightFourPose;
    public PoseObservation limelightThreePose;
  }

  public default void updateInputs(VisionIOInputs inputs) {}
}
