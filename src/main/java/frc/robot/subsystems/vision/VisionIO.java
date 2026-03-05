package frc.robot.subsystems.vision;

import org.littletonrobotics.junction.AutoLog;

public interface VisionIO {
  @AutoLog
  public static class VisionIOInputs {
    public boolean leftConnected = false;
    public boolean rightConnected = false;
    public boolean limelightFourHasTarget;
    public boolean limelightThreeHasTarget;
    public PoseObservation limelightFourMT1Pose;
    public PoseObservation limelightThreeMT1Pose;
    public PoseObservation limelightFourMT2Pose;
    public PoseObservation limelightThreeMT2Pose;
  }

  public default void updateInputs(VisionIOInputs inputs) {}
}
