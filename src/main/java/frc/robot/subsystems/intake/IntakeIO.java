package frc.robot.subsystems.intake;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
  @AutoLog
  public static class IntakeIOInputs {
    // Wheel motor inputs
    public boolean wheelConnected = false;
    public AngularVelocity wheelVelocity = edu.wpi.first.units.Units.RPM.of(0.0);
    public double wheelAppliedVolts = 0.0;
    public double wheelCurrentAmps = 0.0;

    // Wrist motor inputs
    public boolean wristConnected = false;
    public Angle wristPosition = edu.wpi.first.units.Units.Degrees.of(0.0);
    public AngularVelocity wristVelocity = edu.wpi.first.units.Units.DegreesPerSecond.of(0.0);
    public double wristAppliedVolts = 0.0;
    public double wristCurrentAmps = 0.0;
  }

  /** Update current inputs (called from subsystem periodic). */
  public default void updateInputs(IntakeIOInputs inputs) {}

  /** Set wheel motor open loop (volts). */
  public default void setWheelOpenLoop(double volts) {}

  /** Set wheel motor velocity control. */
  public default void setWheelVelocity(AngularVelocity velocity) {}

  /** Set wrist motor open loop (volts). */
  public default void setWristOpenLoop(double volts) {}

  /** Set wrist motor position control. */
  public default void setWristPosition(Angle angle) {}

  /** Zero the wrist encoder. */
  public default void zeroWrist() {}
}
