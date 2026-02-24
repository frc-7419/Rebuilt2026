package frc.robot.subsystems.serializer;

import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotState;
import org.littletonrobotics.junction.Logger;

/**
 * Subsystem with two parts: center wheel (serializer) and feeder (into shooter). Both can be run
 * together or stopped together.
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

  // --------------- Center wheel (serializer) ---------------

  /** Center wheel open-loop voltage. */
  public void setSerializerVoltage(double volts) {
    io.setOpenLoop(volts);
  }

  /** Set serializer wheel target RPM (closed-loop in IO). */
  public void setRPM(double rpm) {
    double targetRadPerSec = RPM.of(rpm).in(RadiansPerSecond);

    double clamped =
        MathUtil.clamp(
            targetRadPerSec,
            SerializerConstants.kMinVelocity.in(RadiansPerSecond),
            SerializerConstants.kMaxVelocity.in(RadiansPerSecond));

    AngularVelocity target = RadiansPerSecond.of(clamped);

    Logger.recordOutput("Serializer/RequestedRadPerSec", clamped);
    Logger.recordOutput("Serializer/RequestedRPM", target.in(RPM));
    io.setVelocity(target);
  }

  /** Stop center wheel. */
  public void stopSerializer() {
    io.setOpenLoop(0.0);
  }

  public AngularVelocity getSerializerVelocity() {
    return inputs.serializerVelocity;
  }

  public double getSerializerRPM() {
    return inputs.serializerVelocity.in(RPM);
  }

  // --------------- Feeder (into shooter) ---------------

  /** Feeder open-loop voltage. */
  public void setFeederVoltage(double volts) {
    io.setFeederOpenLoop(volts);
  }

  /** Feeder closed-loop RPM. */
  public void setFeederRPM(double rpm) {
    double targetRadPerSec = RPM.of(rpm).in(RadiansPerSecond);
    double clamped =
        MathUtil.clamp(
            targetRadPerSec,
            SerializerConstants.kMinVelocity.in(RadiansPerSecond),
            SerializerConstants.kMaxVelocity.in(RadiansPerSecond));
    AngularVelocity target = RadiansPerSecond.of(clamped);
    Logger.recordOutput("Serializer/FeederRequestedRadPerSec", clamped);
    Logger.recordOutput("Serializer/FeederRequestedRPM", target.in(RPM));
    io.setFeederVelocity(target);
  }

  /** Stop feeder. */
  public void stopFeeder() {
    io.setFeederOpenLoop(0.0);
  }

  public AngularVelocity getFeederVelocity() {
    return inputs.feederVelocity;
  }

  public double getFeederRPM() {
    return inputs.feederVelocity.in(RPM);
  }

  // --------------- Both ---------------

  /** Run both at given voltages (e.g. shoot or reverse). */
  public void setBothVoltage(double serializerVolts, double feederVolts) {
    io.setOpenLoop(serializerVolts);
    io.setFeederOpenLoop(feederVolts);
  }

  /** Stop both center wheel and feeder. */
  public void stopBoth() {
    io.setOpenLoop(0.0);
    io.setFeederOpenLoop(0.0);
  }
}
