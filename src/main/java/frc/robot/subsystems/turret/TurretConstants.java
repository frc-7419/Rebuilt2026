// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.turret;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.*;

/** Basic turret configuration values. Update IDs and gains for your robot. */
public final class TurretConstants {
  private TurretConstants() {}

  // Default CAN ID for turret motor (override to match hardware)
  public static final int kTurretMotorId = 9;

  // Control gains (software PID for safety). Tune as needed.
  public static final double kP = 6.0;
  public static final double kI = 0.0;
  public static final double kD = 0.5;

  // Maximum output voltage when using software PID (Volts)
  public static final double kMaxVoltage = 12.0;

  // Allowed motion limits (radians). These are soft limits for safety.
  public static final double kMinAngleRad = -Math.PI;
  public static final double kMaxAngleRad = Math.PI;

  // Small deadband for joystick control
  public static final double kDeadband = 0.05;

  // Conversion helper (if using degrees elsewhere)
  public static double degreesToRad(double deg) {
    return Units.degreesToRadians(deg);
  }
}
