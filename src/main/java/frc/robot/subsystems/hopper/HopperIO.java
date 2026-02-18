package frc.robot.subsystems.hopper;

import edu.wpi.first.units.measure.AngularVelocity;
import org.littletonrobotics.junction.AutoLog;

public interface HopperIO {
  @AutoLog
  public static class HopperIOInputs {
    public boolean connected = false;
    public AngularVelocity velocity = edu.wpi.first.units.Units.RPM.of(0.0);
    public double appliedVolts = 0.0;
    public double currentAmps = 0.0;
  }

  /** Update current inputs (called from subsystem periodic). */
  public default void updateInputs(HopperIOInputs inputs) {}

  /** Set hopper motor in open loop (volts). */
  public default void setOpenLoop(double volts) {}

  /** Set hopper motor velocity control. */
  public default void setVelocity(AngularVelocity velocity) {}
}
