// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.turret.Turret;
import java.util.function.DoubleSupplier;

public final class TurretCommands {
  private TurretCommands() {
  }

  /** Manual joystick turret control. Expects input in [-1, 1]. */
  public static Command joystickTurret(Turret turret, DoubleSupplier input) {
    return Commands.run(
        () -> {
          double val = MathUtil.applyDeadband(input.getAsDouble(), 0.05);
          turret.setOpenLoop(val * 12.0); // scale to volts
        },
        turret);
  }

  /** Hold turret at a specific absolute rotation. */
  public static Command holdAngle(Turret turret, Rotation2d angle) {
    return Commands.runOnce(() -> turret.setAngle(angle), turret).withTimeout(0.0);
  }
}
