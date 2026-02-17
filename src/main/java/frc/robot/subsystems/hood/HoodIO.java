package frc.robot.subsystems.hood;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import org.littletonrobotics.junction.AutoLog;

public interface HoodIO {
  @AutoLog
  public static class HoodIOInputs {
    public boolean connected = false;

    public Angle position = Degrees.of(0.0);
    public AngularVelocity velocity = RadiansPerSecond.of(0.0);
    public double appliedVolts = 0.0;
    public double currentAmps = 0.0;
  }

  /** Update current inputs (called from subsystem periodic). */
  public default void updateInputs(HoodIOInputs inputs) {}

  /** Run hood in open loop (volts). */
  public default void setOpenLoop(double volts) {}

  /** Run hood to target angle */
  public default void setPosition(Angle position) {}

  /** Zero rotor so current physical angle is treated as the offset */
  public default void zeroRotor() {}
}
