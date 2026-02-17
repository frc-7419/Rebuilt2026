package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.hood.Hood;
import java.util.function.DoubleSupplier;

public final class HoodCommands {
  private HoodCommands() {}

  /** Manual joystick turret control. Expects input in [-1, 1]. */
  public static Command joystickHood(Hood hood, DoubleSupplier input) {
    return Commands.run(
        () -> {
          double val = MathUtil.applyDeadband(input.getAsDouble(), 0.05);
          hood.setOpenLoop(val * 1); // scale to volts
        },
        hood);
  }
}
