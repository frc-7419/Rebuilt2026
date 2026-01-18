// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.turret;

import static edu.wpi.first.units.Units.RadiansPerSecond;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotState;
import org.littletonrobotics.junction.Logger;

/** Turret subsystem implementing position control with IO abstraction. */
public class Turret extends SubsystemBase {
  private final TurretIO io;
  private final TurretIOInputsAutoLogged inputs = new TurretIOInputsAutoLogged();

  public Turret(TurretIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Turret", inputs);

    double timestamp = Timer.getFPGATimestamp();
    RobotState.getInstance()
        .addTurretUpdates(
            timestamp, inputs.position, RadiansPerSecond.of(inputs.velocityRadPerSec));
  }

  /** Sets the turret in open loop (volts). Cancels any position hold. */
  public void setOpenLoop(double volts) {
    io.setOpenLoop(volts);
  }

  public void setAngle(Rotation2d angle) {
    double angleRad = angle.getRadians();
    double normalizedAngle = MathUtil.angleModulus(angleRad);

    double wrappedAngle =
        MathUtil.clamp(normalizedAngle, TurretConstants.kMinAngleRad, TurretConstants.kMaxAngleRad);
    Rotation2d target = new Rotation2d(wrappedAngle);

    Logger.recordOutput("Turret/TurretRequestedRad", wrappedAngle);
    io.setPosition(target);
  }

  /** Cancels any position hold and stops the motor. */
  public void stop() {
    io.setOpenLoop(0.0);
  }

  /** Returns the most recent turret angle. */
  public Rotation2d getAngle() {
    return inputs.position;
  }
}
