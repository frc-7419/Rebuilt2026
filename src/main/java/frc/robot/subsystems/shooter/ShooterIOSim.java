package frc.robot.subsystems.shooter;

import static edu.wpi.first.math.MathUtil.*;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static frc.robot.subsystems.shooter.ShooterConstants.*;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

/** Simulation implementation of ShooterIO. */
public class ShooterIOSim implements ShooterIO {
  private static final double SIMULATION_DT = 0.02;

  private static final double SHOOTER_INERTIA = 0.1;
  private static final DCMotor MOTOR_MODEL = DCMotor.getKrakenX60Foc(2);
  private static final double kV =
      12.0 / (MOTOR_MODEL.freeSpeedRadPerSec / kMotorToShooterGearRatio); // Velocity feedforward

  private final DCMotorSim motorSim;
  private final PIDController velocityController;

  private boolean velocityControl = false;

  /** Setpoint on the MECHANISM (wheel) side, in rad/s. */
  private double targetShooterVelocityRadPerSec = 0.0;

  /** Feedforward velocity term on the mechanism side, in rad/s (same structure as turret sim). */
  private double feedforwardVelocityRadPerSec = 0.0;

  private double appliedVolts = 0.0;

  private double rotorVelocityRadPerSec = 0.0;
  private double shooterVelocityRadPerSec = 0.0;
  private double currentAmps = 0.0;

  /** Creates a new ShooterIOSim. */
  public ShooterIOSim() {
    motorSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                MOTOR_MODEL, SHOOTER_INERTIA, kMotorToShooterGearRatio),
            MOTOR_MODEL);

    velocityController = new PIDController(kSimP, kSimI, kSimD);
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    if (velocityControl) {
      double pidOutput =
          velocityController.calculate(shooterVelocityRadPerSec, targetShooterVelocityRadPerSec);
      double feedforwardVolts = feedforwardVelocityRadPerSec * kV;
      appliedVolts = clamp(pidOutput + feedforwardVolts, -kMaxVoltage, kMaxVoltage);
    }

    motorSim.setInputVoltage(appliedVolts);
    motorSim.update(SIMULATION_DT);

    // Motor sim velocity is motor-side; convert to mechanism.
    rotorVelocityRadPerSec = motorSim.getAngularVelocityRadPerSec();
    shooterVelocityRadPerSec = rotorVelocityRadPerSec;

    // Brake damping behavior
    if (Math.abs(appliedVolts) < 0.01) {
      double brakeDamping = 0.8;
      shooterVelocityRadPerSec *= brakeDamping;
      if (Math.abs(shooterVelocityRadPerSec) < 0.01) {
        shooterVelocityRadPerSec = 0.0;
      }
      motorSim.setState(motorSim.getAngularPositionRad(), shooterVelocityRadPerSec);
      rotorVelocityRadPerSec = shooterVelocityRadPerSec;
    }

    currentAmps = Math.abs(motorSim.getCurrentDrawAmps());

    inputs.connected = true;
    inputs.rotorVelocity = RadiansPerSecond.of(rotorVelocityRadPerSec);
    inputs.shooterVelocity = RadiansPerSecond.of(shooterVelocityRadPerSec);
    inputs.requestedVelocity = RadiansPerSecond.of(targetShooterVelocityRadPerSec);
    inputs.appliedVolts = appliedVolts;
    inputs.currentAmps = currentAmps;
  }

  @Override
  public void setOpenLoop(double volts) {
    velocityControl = false;
    velocityController.reset();
    feedforwardVelocityRadPerSec = 0.0;
    appliedVolts = clamp(volts, -kMaxVoltage, kMaxVoltage);
  }

  @Override
  public void setVelocity(AngularVelocity velocity) {
    velocityControl = true;
    feedforwardVelocityRadPerSec = 0.0;

    double targetRadPerSec = velocity.in(RadiansPerSecond);
    targetRadPerSec =
        clamp(
            targetRadPerSec, kMinVelocity.in(RadiansPerSecond), kMaxVelocity.in(RadiansPerSecond));
    targetShooterVelocityRadPerSec = targetRadPerSec;
  }

  @Override
  public void zeroRotor() {
    motorSim.setState(0.0, 0.0);
    targetShooterVelocityRadPerSec = 0.0;
    feedforwardVelocityRadPerSec = 0.0;
    appliedVolts = 0.0;
    velocityController.reset();
  }
}
