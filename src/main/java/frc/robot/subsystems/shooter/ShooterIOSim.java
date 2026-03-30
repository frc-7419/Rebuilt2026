package frc.robot.subsystems.shooter;

import static edu.wpi.first.math.MathUtil.clamp;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static frc.robot.subsystems.shooter.ShooterConstants.computeVelocityVolts;
import static frc.robot.subsystems.shooter.ShooterConstants.kMaxVoltage;
import static frc.robot.subsystems.shooter.ShooterConstants.kMotorToShooterGearRatio;
import static frc.robot.subsystems.shooter.ShooterConstants.kSimKp;
import static frc.robot.subsystems.shooter.ShooterConstants.kSimKv;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

/** Simulation implementation of ShooterIO. */
public class ShooterIOSim implements ShooterIO {
  private static final double SIMULATION_DT = 0.02;

  private static final double SHOOTER_INERTIA = 0.1;
  private static final DCMotor MOTOR_MODEL = DCMotor.getKrakenX60Foc(2);

  private final DCMotorSim motorSim;

  private double appliedVolts = 0.0;
  /** Target mechanism velocity (rad/s); NaN when open-loop only. */
  private double targetVelocityRadPerSec = Double.NaN;

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
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    if (Double.isFinite(targetVelocityRadPerSec)) {
      double targetRps = RadiansPerSecond.of(targetVelocityRadPerSec).in(RotationsPerSecond);
      double actualRps = RadiansPerSecond.of(shooterVelocityRadPerSec).in(RotationsPerSecond);
      appliedVolts =
          clamp(
              computeVelocityVolts(targetRps, actualRps, kSimKv, kSimKp),
              -kMaxVoltage,
              kMaxVoltage);
    }

    motorSim.setInputVoltage(appliedVolts);
    motorSim.update(SIMULATION_DT);

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
    inputs.requestedVelocity =
        Double.isFinite(targetVelocityRadPerSec)
            ? RadiansPerSecond.of(targetVelocityRadPerSec)
            : RadiansPerSecond.of(0.0);
    inputs.appliedVolts = appliedVolts;
    inputs.currentAmps = currentAmps;
  }

  @Override
  public void setOpenLoop(double volts) {
    targetVelocityRadPerSec = Double.NaN;
    appliedVolts = clamp(volts, -kMaxVoltage, kMaxVoltage);
  }

  @Override
  public void setVelocity(AngularVelocity velocity) {
    targetVelocityRadPerSec = velocity.in(RadiansPerSecond);
  }

  @Override
  public void zeroRotor() {
    motorSim.setState(0.0, 0.0);
    appliedVolts = 0.0;
    targetVelocityRadPerSec = Double.NaN;
  }
}
