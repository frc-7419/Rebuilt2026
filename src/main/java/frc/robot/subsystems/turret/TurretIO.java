// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.turret;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Rotations;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import org.littletonrobotics.junction.AutoLog;

public interface TurretIO {
  @AutoLog
  public static class TurretIOInputs {
    public boolean connected = false;
    public Angle absolutePosition = Degrees.of(0.0);
    public Angle rotorPosition = Degrees.of(0.0);
    public Angle encoderOnePosition = Degrees.of(0.0);
    public Angle encoderTwoPosition = Degrees.of(0.0);
    public Angle encoderOneZeroOffset = Rotations.of(0);
    public Angle encoderTwoZeroOffset = Rotations.of(0);
    public double velocityRadPerSec = 0.0;
    public double appliedVolts = 0.0;
    public double currentAmps = 0.0;
  }

  /** Update current inputs (called from subsystem periodic). */
  public default void updateInputs(TurretIOInputs inputs) {}

  /** Run turret in open loop using a voltage (Volts). */
  public default void setOpenLoop(double volts) {}

  /** Run turret to a specific rotation (radians via Rotation2d). */
  public default void setPosition(Rotation2d position) {}
}
