package frc.robot.commands;

import static edu.wpi.first.math.MathUtil.applyDeadband;
import static edu.wpi.first.wpilibj2.command.Commands.run;
import static edu.wpi.first.wpilibj2.command.Commands.runOnce;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.turret.Turret;
import java.util.function.DoubleSupplier;

public final class TurretCommands {
  private TurretCommands() {}

  /** Manual joystick turret control. Expects input in [-1, 1]. */
  public static Command joystickTurret(Turret turret, DoubleSupplier input) {
    return run(
        () -> {
          double val = applyDeadband(input.getAsDouble(), 0.05);
          turret.setOpenLoop(val * 12.0); // scale to volts
        },
        turret);
  }

  /** Hold turret at a specific absolute rotation. */
  public static Command holdAngle(Turret turret, Angle angle) {
    return runOnce(() -> turret.setAngle(angle), turret).withTimeout(0.0);
  }

  /** Run turret to a specific position. */
  public static Command toTurretPosition(Turret turret, Angle position) {
    return runOnce(() -> turret.setAngle(position), turret);
  }
}
