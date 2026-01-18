// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.turret;

import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

/** Basic turret configuration values. Update IDs and gains for your robot. */
public final class TurretConstants {
  private TurretConstants() {}

  // Default CAN ID for turret motor (override to match hardware)
  public static final int kTurretMotorId = 9;

  // Control gains (software PID for safety). Tune as needed.
  public static final double kP = 40;
  public static final double kI = 0.0;
  public static final double kD = 10;

  // Maximum output voltage when using software PID (Volts)
  public static final double kMaxVoltage = 12.0;

  // Maximum rotation range in degreess
  public static final double kTurretMaxRotations = 270.0;

  // Allowed motion limits (radians).
  public static final double kMinAngleRad = Units.degreesToRadians(-kTurretMaxRotations / 2.0);
  public static final double kMaxAngleRad = Units.degreesToRadians(kTurretMaxRotations / 2.0);

  // Small deadband for joystick control
  public static final double kDeadband = 0.05;

  // Gear ratio between motor and turret (change)
  public static final double kMotorToTurretGearRatio = 0.4;

  // Turret pivot point offset from robot center (forward, left)
  // Positive forward = forward of robot center, positive left = left of robot center
  public static final Transform2d kTurretOffset =
      new Transform2d(new Translation2d(0.0, 0.0), new edu.wpi.first.math.geometry.Rotation2d());
}
