package frc.robot.commands;

import static edu.wpi.first.math.MathUtil.*;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.wpilibj2.command.Commands.*;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.Intake;
import java.util.function.DoubleSupplier;

public final class IntakeCommands {
  private IntakeCommands() {}

  /** Manual joystick wheel control. Expects input in [-1, 1]. */
  public static Command joystickWheel(Intake intake, DoubleSupplier input) {
    return run(
        () -> {
          double val = applyDeadband(input.getAsDouble(), 0.05);
          intake.setWheelOpenLoop(val * 12.0); // scale to volts
        },
        intake);
  }

  /** Manual joystick wrist control. Expects input in [-1, 1]. */
  public static Command joystickWrist(Intake intake, DoubleSupplier input) {
    return run(
        () -> {
          double val = applyDeadband(input.getAsDouble(), 0.05);
          intake.setWristOpenLoop(val * 4.0); // scale to volts
        },
        intake);
  }

  /** Run intake wheel at open loop voltage. */
  public static Command runWheel(Intake intake, double volts) {
    return run(() -> intake.setWheelOpenLoop(volts), intake);
  }

  public static Command runWheelWithVelocity(Intake intake, AngularVelocity vel) {
    return run(() -> intake.setWheelVelocity(vel), intake);
  }

  /** Set intake wrist to a specific angle. */
  public static Command setWristAngle(Intake intake, Angle angle) {
    return runOnce(() -> intake.setWristAngle(angle), intake);
  }

  public static Command setWristAngleWiggle(Intake intake, Angle midpoint, Angle wiggle) {
    return run(
        () ->
            intake.setWristAngle(
                Degrees.of(
                    wiggle.in(Degrees) * Math.sin(Timer.getTimestamp() * 5)
                        + midpoint.in(Degrees))),
        intake);
  }

  /** Stop both wheel and wrist. */
  public static Command stop(Intake intake) {
    return runOnce(
        () -> {
          intake.stopWheel();
          intake.stopWrist();
        },
        intake);
  }
}
