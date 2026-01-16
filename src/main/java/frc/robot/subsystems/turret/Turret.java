// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.turret;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

/** Turret subsystem implementing simple PID position control with IO abstraction. */
public class Turret extends SubsystemBase {
  private final TurretIO io;
  private final TurretIOInputsAutoLogged inputs = new TurretIOInputsAutoLogged();
  private final PIDController pid;

  // Target angle in radians
  private Rotation2d target = Rotation2d.kZero;
  private boolean holding = false;

  public Turret(TurretIO io) {
    this.io = io;
    pid = new PIDController(TurretConstants.kP, TurretConstants.kI, TurretConstants.kD);
    pid.enableContinuousInput(-Math.PI, Math.PI);
    pid.setTolerance(Units.radiansToDegrees(0.5));
  }

  @Override
  public void periodic() {
    // Update and log inputs
    io.updateInputs(inputs);
    Logger.processInputs("Turret", inputs);

    // If we're holding a setpoint, use PID to compute a voltage command
    if (holding) {
      double measurement = inputs.position.getRadians();
      double error = target.getRadians();
      double volts = pid.calculate(measurement, error);
      // Clamp output
      if (volts > TurretConstants.kMaxVoltage) volts = TurretConstants.kMaxVoltage;
      if (volts < -TurretConstants.kMaxVoltage) volts = -TurretConstants.kMaxVoltage;
      io.setOpenLoop(volts);
    }
  }

  /** Sets the turret in open loop (volts). Cancels any position hold. */
  public void setOpenLoop(double volts) {
    holding = false;
    io.setOpenLoop(volts);
  }

  /** Sets a target angle to hold (Rotation2d). */
  public void setAngle(Rotation2d angle) {
    // Clamp to limits
    double r =
        Math.max(
            TurretConstants.kMinAngleRad,
            Math.min(TurretConstants.kMaxAngleRad, angle.getRadians()));
    target = new Rotation2d(r);
    pid.reset();
    pid.setSetpoint(target.getRadians());
    holding = true;
  }

  /** Cancels any position hold and stops the motor. */
  public void stop() {
    holding = false;
    io.setOpenLoop(0.0);
  }

  /** Returns the most recent turret angle. */
  public Rotation2d getAngle() {
    return inputs.position;
  }
}
