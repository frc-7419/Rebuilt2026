package frc.robot.subsystems.serializer;

import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.units.measure.AngularVelocity;
import org.littletonrobotics.junction.AutoLog;

public interface SerializerIO {
  @AutoLog
  public static class SerializerIOInputs {
    // Serializer wheel (main mechanism)
    public boolean serializerConnected = false;
    public AngularVelocity serializerVelocity = RPM.of(0.0);
    public double serializerAppliedVolts = 0.0;
    public double serializerCurrentAmps = 0.0;

    // Feeder rollers (separate motor)
    public boolean feederConnected = false;
    public AngularVelocity feederVelocity = RPM.of(0.0);
    public double feederAppliedVolts = 0.0;
    public double feederCurrentAmps = 0.0;
  }

  /** Update current inputs (called from subsystem periodic). */
  public default void updateInputs(SerializerIOInputs inputs) {}

  /** Set serializer wheel motor in open loop (volts). */
  public default void setSerializerOpenLoop(double volts) {}

  /** Set serializer wheel motor velocity control. */
  public default void setVelocity(AngularVelocity velocity) {}

  /** Set feeder rollers in open loop (volts). */
  public default void setFeederOpenLoop(double volts) {}

  /** Set feeder rollers velocity control. */
  public default void setFeederVelocity(AngularVelocity velocity) {}
}
