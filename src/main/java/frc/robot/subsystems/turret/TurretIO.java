// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.turret;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface TurretIO {
  @AutoLog
  public static class TurretIOInputs {
    public boolean connected = false;
    public Rotation2d absolutePosition = Rotation2d.kZero;
    public Rotation2d position = Rotation2d.kZero;
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
