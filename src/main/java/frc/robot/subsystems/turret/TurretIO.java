package frc.robot.subsystems.turret;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import org.littletonrobotics.junction.AutoLog;

public interface TurretIO {
  @AutoLog
  public static class TurretIOInputs {
    public boolean connected = false;
    public Angle rotorPosition = Degrees.of(0.0);
    public Angle turretPosition = Degrees.of(0.0);
    public Angle rightEncoderPosition = Degrees.of(0.0);
    public Angle leftEncoderPosition = Degrees.of(0.0);
    public Angle rightEncoderZeroOffset = Rotations.of(0);
    public Angle leftEncoderZeroOffset = Rotations.of(0);
    public AngularVelocity velocity = RadiansPerSecond.of(0.0);
    public Angle requestedPosition = Rotations.of(0);
    public AngularVelocity requestedVelocity = RotationsPerSecond.of(0);
    public double appliedVolts = 0.0;
    public double currentAmps = 0.0;
  }

  /** Update current inputs (called from subsystem periodic). */
  public default void updateInputs(TurretIOInputs inputs) {}

  /** Run turret in open loop using a voltage (Volts). */
  public default void setOpenLoop(double volts) {}

  /** Run turret to a specific Angle */
  public default void setPosition(Angle position) {}

  /** Run turret to a specific state */
  public default void setState(Angle position, AngularVelocity velocity) {}

  /** Zero the rotor position to a specific offset. */
  public default void zeroRotor(Angle offset) {}
}
