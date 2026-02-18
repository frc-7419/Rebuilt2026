package frc.robot.subsystems.serializer;

import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotState;
import org.littletonrobotics.junction.Logger;

/**
 * Serializer subsystem containing the serializer wheel and feeder rollers for feeding fuel into the
 * turret.
 */
public class Serializer extends SubsystemBase {
  private final SerializerIO io;
  private final SerializerIOInputsAutoLogged inputs = new SerializerIOInputsAutoLogged();

  public Serializer(SerializerIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Serializer", inputs);

    double timestamp = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();
    RobotState.getInstance().addHopperUpdates(timestamp, inputs.serializerVelocity);
  }

  // ==================== Serializer Wheel Methods ====================

  /** Set serializer wheel in open loop (volts). Cancels any velocity hold. */
  public void setOpenLoop(double volts) {
    io.setOpenLoop(volts);
  }

  /** Set serializer wheel target RPM (closed-loop in IO). */
  public void setRPM(double rpm) {
    double targetRadPerSec = RPM.of(rpm).in(edu.wpi.first.units.Units.RadiansPerSecond);

    double clamped =
        MathUtil.clamp(
            targetRadPerSec,
            SerializerConstants.kMinVelocity.in(edu.wpi.first.units.Units.RadiansPerSecond),
            SerializerConstants.kMaxVelocity.in(edu.wpi.first.units.Units.RadiansPerSecond));

    AngularVelocity target = edu.wpi.first.units.Units.RadiansPerSecond.of(clamped);

    Logger.recordOutput("Serializer/RequestedRadPerSec", clamped);
    Logger.recordOutput("Serializer/RequestedRPM", target.in(RPM));

    io.setVelocity(target);
  }

  /** Cancels any velocity hold and stops the serializer wheel. */
  public void stop() {
    io.setOpenLoop(0.0);
  }

  /** Returns the most recent serializer wheel velocity. */
  public AngularVelocity getVelocity() {
    return inputs.serializerVelocity;
  }

  /** Convenience accessor in RPM. */
  public double getRPM() {
    return inputs.serializerVelocity.in(RPM);
  }

  // ==================== Feeder Roller Methods ====================

  /** Set feeder rollers in open loop (volts). */
  public void setFeederOpenLoop(double volts) {
    io.setFeederOpenLoop(volts);
  }

  /** Set feeder rollers target RPM (closed-loop in IO). */
  public void setFeederRPM(double rpm) {
    double targetRadPerSec = RPM.of(rpm).in(edu.wpi.first.units.Units.RadiansPerSecond);

    double clamped =
        MathUtil.clamp(
            targetRadPerSec,
            SerializerConstants.kMinVelocity.in(edu.wpi.first.units.Units.RadiansPerSecond),
            SerializerConstants.kMaxVelocity.in(edu.wpi.first.units.Units.RadiansPerSecond));

    AngularVelocity target = edu.wpi.first.units.Units.RadiansPerSecond.of(clamped);

    Logger.recordOutput("Serializer/FeederRequestedRadPerSec", clamped);
    Logger.recordOutput("Serializer/FeederRequestedRPM", target.in(RPM));

    io.setFeederVelocity(target);
  }

  /** Stop the feeder rollers. */
  public void stopFeeder() {
    io.setFeederOpenLoop(0.0);
  }

  /** Returns the most recent feeder roller velocity. */
  public AngularVelocity getFeederVelocity() {
    return inputs.feederVelocity;
  }

  /** Convenience accessor in RPM for feeder. */
  public double getFeederRPM() {
    return inputs.feederVelocity.in(RPM);
  }
}
