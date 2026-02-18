package frc.robot.commands;

import static edu.wpi.first.math.MathUtil.*;
import static edu.wpi.first.wpilibj2.command.Commands.*;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.hood.Hood;
import java.util.function.DoubleSupplier;

public final class HoodCommands {
  private HoodCommands() {}

  /** Manual joystick turret control. Expects input in [-1, 1]. */
  public static Command joystickHood(Hood hood, DoubleSupplier input) {
    return run(
        () -> {
          double val = applyDeadband(input.getAsDouble(), 0.05);
          hood.setOpenLoop(val * 3); // scale to volts
        },
        hood);
  }
}
