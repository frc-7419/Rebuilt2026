package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.hopper.Hopper;
import java.util.function.DoubleSupplier;

public final class HopperCommands {
  private HopperCommands() {}

  /** Manual joystick hopper control. Expects input in [-1, 1]. */
  public static Command joystickHopper(Hopper hopper, DoubleSupplier input) {
    return Commands.run(
        () -> {
          double val = MathUtil.applyDeadband(input.getAsDouble(), 0.05);
          hopper.setOpenLoop(val * 12.0); // scale to volts
        },
        hopper);
  }

  /** Set hopper to run at a constant RPM. */
  public static Command setRPM(Hopper hopper, double rpm) {
    return Commands.run(() -> hopper.setRPM(rpm), hopper);
  }

  /** Run hopper at open loop voltage. */
  public static Command runVoltage(Hopper hopper, double volts) {
    return Commands.run(() -> hopper.setOpenLoop(volts), hopper);
  }

  /** Stop the hopper. */
  public static Command stop(Hopper hopper) {
    return Commands.runOnce(hopper::stop, hopper);
  }
}
