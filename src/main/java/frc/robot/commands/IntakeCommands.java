package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.intake.Intake;
import java.util.function.DoubleSupplier;

public final class IntakeCommands {
  private IntakeCommands() {}

  /** Manual joystick wheel control. Expects input in [-1, 1]. */
  public static Command joystickWheel(Intake intake, DoubleSupplier input) {
    return Commands.run(
        () -> {
          double val = MathUtil.applyDeadband(input.getAsDouble(), 0.05);
          intake.setWheelOpenLoop(val * 12.0); // scale to volts
        },
        intake);
  }

  /** Manual joystick wrist control. Expects input in [-1, 1]. */
  public static Command joystickWrist(Intake intake, DoubleSupplier input) {
    return Commands.run(
        () -> {
          double val = MathUtil.applyDeadband(input.getAsDouble(), 0.05);
          intake.setWristOpenLoop(val * 12.0); // scale to volts
        },
        intake);
  }

  /** Run intake wheel at open loop voltage. */
  public static Command runWheel(Intake intake, double volts) {
    return Commands.run(() -> intake.setWheelOpenLoop(volts), intake);
  }

  /** Set intake wrist to a specific angle. */
  public static Command setWristAngle(Intake intake, Angle angle) {
    return Commands.runOnce(() -> intake.setWristAngle(angle), intake)
        .andThen(Commands.idle(intake));
  }

  /** Stop both wheel and wrist. */
  public static Command stop(Intake intake) {
    return Commands.runOnce(
        () -> {
          intake.stopWheel();
          intake.stopWrist();
        },
        intake);
  }
}
