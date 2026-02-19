package frc.robot.commands;

import static edu.wpi.first.math.MathUtil.*;
import static edu.wpi.first.wpilibj2.command.Commands.*;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.serializer.Serializer;
import java.util.function.DoubleSupplier;

public final class SerializerCommands {
  private SerializerCommands() {}

  // ==================== Serializer Wheel Commands ====================

  /** Manual joystick serializer wheel control. Expects input in [-1, 1]. */
  public static Command joystickSerializer(Serializer serializer, DoubleSupplier input) {
    return run(
        () -> {
          double val = applyDeadband(input.getAsDouble(), 0.05);
          serializer.setOpenLoop(val * 12.0); // scale to volts
        },
        serializer);
  }

  /** Set serializer wheel to run at a constant RPM. */
  public static Command setRPM(Serializer serializer, double rpm) {
    return run(() -> serializer.setRPM(rpm), serializer);
  }

  /** Run serializer wheel at open loop voltage. */
  public static Command runVoltage(Serializer serializer, double volts) {
    return run(() -> serializer.setOpenLoop(volts), serializer);
  }

  /** Stop the serializer wheel. */
  public static Command stop(Serializer serializer) {
    return runOnce(serializer::stop, serializer);
  }

  // ==================== Feeder Roller Commands ====================

  /** Manual joystick feeder roller control. Expects input in [-1, 1]. */
  public static Command joystickFeeder(Serializer serializer, DoubleSupplier input) {
    return run(
        () -> {
          double val = applyDeadband(input.getAsDouble(), 0.05);
          serializer.setFeederOpenLoop(val * 12.0); // scale to volts
        },
        serializer);
  }

  /** Set feeder rollers to run at a constant RPM. */
  public static Command setFeederRPM(Serializer serializer, double rpm) {
    return run(() -> serializer.setFeederRPM(rpm), serializer);
  }

  /** Run feeder rollers at open loop voltage. */
  public static Command runFeederVoltage(Serializer serializer, double volts) {
    return run(() -> serializer.setFeederOpenLoop(volts), serializer);
  }

  /** Stop the feeder rollers. */
  public static Command stopFeeder(Serializer serializer) {
    return runOnce(serializer::stopFeeder, serializer);
  }
}
