// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;

public class ShooterConstants {
  public static final Angle kHoodZeroed = Degrees.of(0.0);

  // Fuel launch velocity per wheel rotation per second (m/s per rot/s)
  // Replace with experiment
  public static final double kFuelLaunchVelMetersPerSecPerRotPerSec = 0.287;

  public static final Distance kShooterWheelRadius = Inches.of(2.0);
  // Spin transfer efficiency from wheel to fuel (0.0 to 1.0)
  public static final double kSpinTransfer = 0.6;

  // Shooter release point relative to robot center (forward, left, up in meters)
  public static final Transform3d kRobotToShooterRelease =
      new Transform3d(new Translation3d(0.0, 0.0, 0.5), new Rotation3d(0.0, 0.0, 0.0));
}
