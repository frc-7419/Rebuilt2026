package frc.robot.subsystems.shooter;

import static edu.wpi.first.math.MathUtil.*;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.wpilibj.Timer.getFPGATimestamp;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotState;
import org.littletonrobotics.junction.Logger;

/** Shooter subsystem implementing velocity control with IO abstraction. */
public class Shooter extends SubsystemBase {
  private final ShooterIO io;
  private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

  /** Last RPM passed to {@link #setRPM(double)}. */
  private double requestedRpm = 0.0;

  public Shooter(ShooterIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Shooter", inputs);

    double timestamp = getFPGATimestamp();
    RobotState.getInstance()
        .addShooterUpdates(timestamp, inputs.shooterVelocity, inputs.rotorVelocity);
  }

  /** Sets the shooter in open loop (volts). Cancels any velocity hold. */
  public void setOpenLoop(double volts) {
    io.setOpenLoop(volts);
  }

  /** Set shooter target velocity. Clamped to constants. */
  public void setVelocity(AngularVelocity velocity) {
    double targetRadPerSec = velocity.in(RadiansPerSecond);
    double clamped =
        clamp(
            targetRadPerSec,
            ShooterConstants.kMinVelocity.in(RadiansPerSecond),
            ShooterConstants.kMaxVelocity.in(RadiansPerSecond));
    AngularVelocity target = RadiansPerSecond.of(clamped);
    requestedRpm = target.in(RPM);

    Logger.recordOutput("Shooter/RequestedRPM", target.in(RPM));
    io.setVelocity(target);
  }

  /** Returns the last requested RPM (set via {@link #setRPM(double)}). */
  public double getRequestedRPM() {
    return requestedRpm;
  }

  /** Cancels any velocity hold and stops the motor. */
  public void stop() {
    io.setOpenLoop(0.0);
  }

  /** Returns the most recent shooter wheel velocity. */
  public AngularVelocity getVelocity() {
    return inputs.shooterVelocity;
  }

  public double getRPM() {
    return inputs.shooterVelocity.in(RPM);
  }
}
