// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.shooter.Shooter;
import java.util.function.DoubleSupplier;

public final class ShooterCommands {
  private ShooterCommands() {}

  /** Manual joystick turret control. Expects input in [-1, 1]. */
  public static Command joystickShooter(Shooter shooter, DoubleSupplier input) {
    return Commands.run(
        () -> {
          double val = MathUtil.applyDeadband(input.getAsDouble(), 0.05);
          shooter.setOpenLoop(val * 12.0); // scale to volts
        },
        shooter);
  }
}
