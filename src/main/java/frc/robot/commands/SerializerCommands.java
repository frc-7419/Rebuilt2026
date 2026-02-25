package frc.robot.commands;

import static edu.wpi.first.math.MathUtil.applyDeadband;
import static edu.wpi.first.wpilibj2.command.Commands.run;
import static edu.wpi.first.wpilibj2.command.Commands.runOnce;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.serializer.Serializer;
import java.util.function.DoubleSupplier;

public final class SerializerCommands {
  private SerializerCommands() {}

  // --------------- Center wheel (serializer) ---------------

  /** Manual joystick control for center wheel. Input in [-1, 1]. */
  public static Command joystickSerializer(Serializer serializer, DoubleSupplier input) {
    return run(
        () -> serializer.setSerializerVoltage(applyDeadband(input.getAsDouble(), 0.05) * 12.0),
        serializer);
  }

  /** Center wheel at constant RPM. */
  public static Command setSerializerRPM(Serializer serializer, double rpm) {
    return run(() -> serializer.setSerializerRPM(rpm), serializer);
  }

  /** Center wheel at open-loop voltage. */
  public static Command runSerializerVoltage(Serializer serializer, double volts) {
    return run(() -> serializer.setSerializerVoltage(volts), serializer);
  }

  /** Stop center wheel. */
  public static Command stopSerializer(Serializer serializer) {
    return runOnce(serializer::stopSerializer, serializer);
  }

  // --------------- Feeder ---------------

  /** Manual joystick control for feeder. Input in [-1, 1]. */
  public static Command joystickFeeder(Serializer serializer, DoubleSupplier input) {
    return run(
        () -> serializer.setFeederVoltage(applyDeadband(input.getAsDouble(), 0.05) * 12.0),
        serializer);
  }

  /** Feeder at constant RPM. */
  public static Command setFeederRPM(Serializer serializer, double rpm) {
    return run(() -> serializer.setFeederRPM(rpm), serializer);
  }

  /** Feeder at open-loop voltage. */
  public static Command runFeederVoltage(Serializer serializer, double volts) {
    return run(() -> serializer.setFeederVoltage(volts), serializer);
  }

  /** Stop feeder. */
  public static Command stopFeeder(Serializer serializer) {
    return runOnce(serializer::stopFeeder, serializer);
  }

  // --------------- Both ---------------

  /** Run both at given voltages (e.g. shoot forward or reverse). */
  public static Command runBothVoltage(
      Serializer serializer, double serializerVolts, double feederVolts) {
    return run(() -> serializer.setBothVoltage(serializerVolts, feederVolts), serializer);
  }

  /** Stop both center wheel and feeder. */
  public static Command stopBoth(Serializer serializer) {
    return runOnce(serializer::stopBoth, serializer);
  }
}
