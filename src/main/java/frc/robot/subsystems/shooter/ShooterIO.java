package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.units.measure.AngularVelocity;
import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
  @AutoLog
  public static class ShooterIOInputs {
    public boolean connected = false;

    /** Motor-side velocity (what the motor sensor reports). */
    public AngularVelocity rotorVelocity = RPM.of(0.0);

    /** Mechanism (wheel) velocity after gear ratio. */
    public AngularVelocity shooterVelocity = RPM.of(0.0);

    public double appliedVolts = 0.0;
    public double currentAmps = 0.0;
  }

  /** Update current inputs (called from subsystem periodic). */
  public default void updateInputs(ShooterIOInputs inputs) {}

  /** Run shooter in open loop using a voltage (Volts). */
  public default void setOpenLoop(double volts) {}

  /** Run shooter to a specific velocity. */
  public default void setVelocity(AngularVelocity velocity) {}

  /** Zero/seed sensor state if desired (optional for shooter). */
  public default void zeroRotor() {}
}
