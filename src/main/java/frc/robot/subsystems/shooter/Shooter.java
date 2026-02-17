package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotState;
import org.littletonrobotics.junction.Logger;

/** Shooter subsystem implementing velocity control with IO abstraction. */
public class Shooter extends SubsystemBase {
  private final ShooterIO io;
  private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

  public Shooter(ShooterIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Shooter", inputs);

    double timestamp = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();
    RobotState.getInstance()
        .addShooterUpdates(timestamp, inputs.shooterVelocity, inputs.rotorVelocity);
  }

  /** Sets the shooter in open loop (volts). Cancels any velocity hold. */
  public void setOpenLoop(double volts) {
    io.setOpenLoop(volts);
  }

  /** Set shooter target RPM (closed-loop in IO). */
  public void setRPM(double rpm) {
    double targetRadPerSec = RPM.of(rpm).in(RadiansPerSecond);

    double clamped =
        MathUtil.clamp(
            targetRadPerSec,
            ShooterConstants.kMinVelocity.in(RadiansPerSecond),
            ShooterConstants.kMaxVelocity.in(RadiansPerSecond));

    AngularVelocity target = RadiansPerSecond.of(clamped);

    Logger.recordOutput("Shooter/RequestedRadPerSec", clamped);
    Logger.recordOutput("Shooter/RequestedRPM", target.in(RPM));

    io.setVelocity(target);
  }

  /** Cancels any velocity hold and stops the motor. */
  public void stop() {
    io.setOpenLoop(0.0);
  }

  /** Returns the most recent shooter wheel velocity. */
  public AngularVelocity getVelocity() {
    return inputs.shooterVelocity;
  }

  /** Convenience accessor in RPM. */
  public double getRPM() {
    return inputs.shooterVelocity.in(RPM);
  }
}
