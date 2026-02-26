package frc.robot.commands;

import static edu.wpi.first.math.MathUtil.*;
import static edu.wpi.first.wpilibj2.command.Commands.*;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.Shooter;
import java.util.function.DoubleSupplier;

public final class ShooterCommands {
  private ShooterCommands() {}

  /** Manual joystick turret control. Expects input in [-1, 1]. */
  public static Command joystickShooter(Shooter shooter, DoubleSupplier input) {
    return run(
        () -> {
          double val = applyDeadband(input.getAsDouble(), 0.05);
          shooter.setOpenLoop(val * 12.0); // scale to volts
        },
        shooter);
  }

  public static Command velocityShooter(Shooter shooter, AngularVelocity velocity) {
    return run(() -> shooter.setVelocity(velocity), shooter);
  }
}
