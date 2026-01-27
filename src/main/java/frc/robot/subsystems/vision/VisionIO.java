package frc.robot.subsystems.vision;

import org.littletonrobotics.junction.AutoLog;

public interface VisionIO {
  @AutoLog
  public static class VisionIOInputs {
    public boolean connected = false;
    public boolean leftHasTarget;
    public boolean rightHasTarget;
    public PoseObservation leftPose;
    public PoseObservation rightPose;
  }

  public default void updateInputs(VisionIOInputs inputs) {}
}
