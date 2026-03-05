// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotState;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.turret.Turret;

public final class ManualAutonCommands {
  public static Command driveAndAutoAim(Drive drive, Turret turret, Shooter shooter, Hood hood) {
    RobotState state = RobotState.getInstance();
    return Commands.sequence(
        DriveCommands.driveFixedVelocity(drive, 0.5, 0.0, 0.0).withTimeout(2.0),
        Commands.runOnce(() -> state.setAutoAimEnabled(true)),
        Commands.runOnce(() -> state.setHubMode(true)));
  }
}
