package frc.robot.subsystems.hopper;

import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotState;
import org.littletonrobotics.junction.Logger;

/** Hopper (serializer) subsystem for feeding fuel into the turret. */
public class Hopper extends SubsystemBase {
  private final HopperIO io;
  private final HopperIOInputsAutoLogged inputs = new HopperIOInputsAutoLogged();

  public Hopper(HopperIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Hopper", inputs);

    double timestamp = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();
    RobotState.getInstance().addHopperUpdates(timestamp, inputs.velocity);
  }

  /** Set hopper in open loop (volts). Cancels any velocity hold. */
  public void setOpenLoop(double volts) {
    io.setOpenLoop(volts);
  }

  /** Set hopper target RPM (closed-loop in IO). */
  public void setRPM(double rpm) {
    double targetRadPerSec = RPM.of(rpm).in(edu.wpi.first.units.Units.RadiansPerSecond);

    double clamped =
        MathUtil.clamp(
            targetRadPerSec,
            HopperConstants.kMinVelocity.in(edu.wpi.first.units.Units.RadiansPerSecond),
            HopperConstants.kMaxVelocity.in(edu.wpi.first.units.Units.RadiansPerSecond));

    AngularVelocity target = edu.wpi.first.units.Units.RadiansPerSecond.of(clamped);

    Logger.recordOutput("Hopper/RequestedRadPerSec", clamped);
    Logger.recordOutput("Hopper/RequestedRPM", target.in(RPM));

    io.setVelocity(target);
  }

  /** Cancels any velocity hold and stops the motor. */
  public void stop() {
    io.setOpenLoop(0.0);
  }

  /** Returns the most recent hopper wheel velocity. */
  public AngularVelocity getVelocity() {
    return inputs.velocity;
  }

  /** Convenience accessor in RPM. */
  public double getRPM() {
    return inputs.velocity.in(RPM);
  }
}
