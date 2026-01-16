package frc.robot.subsystems.vision;

import org.littletonrobotics.junction.AutoLog;

public interface VisionIO {
  @AutoLog
  public static class VisionIOInputs {
    public boolean connected = false;
    public boolean turretHasTarget;
    public boolean supplementaryHasTarget;
    public PoseObservation turretPose;
    public PoseObservation supplementaryPose;
  }

  public default void updateInputs(VisionIOInputs inputs) {
  }
}
