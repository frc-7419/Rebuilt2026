package frc.robot.subsystems.shooter;

import static edu.wpi.first.math.MathUtil.clamp;
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
  private static final double TWO_PI = 2.0 * Math.PI;
  private static final double MOTOR_TO_SHOOTER_REDUCTION = 1.0 / kMotorToShooterGearRatio;

  private static final double SHOOTER_INERTIA = 0.1;
  private static final DCMotor MOTOR_MODEL = DCMotor.getKrakenX60Foc(2);

  private final DCMotorSim motorSim;

  private double appliedVolts = 0.0;
  /** Target mechanism velocity (rot/s); NaN when open-loop only. */
  private double targetVelocityRotPerSec = Double.NaN;

  private double rotorVelocityRotPerSec = 0.0;
  private double shooterVelocityRotPerSec = 0.0;
  private double currentAmps = 0.0;

  /** Creates a new ShooterIOSim. */
  public ShooterIOSim() {
    motorSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                MOTOR_MODEL, SHOOTER_INERTIA, MOTOR_TO_SHOOTER_REDUCTION),
            MOTOR_MODEL);
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    if (Double.isFinite(targetVelocityRotPerSec)) {
      appliedVolts =
          clamp(
              computeVelocityVolts(
                  targetVelocityRotPerSec, shooterVelocityRotPerSec, kSimKv, kSimKp),
              -kMaxVoltage,
              kMaxVoltage);
    }

    motorSim.setInputVoltage(appliedVolts);
    motorSim.update(SIMULATION_DT);

    shooterVelocityRotPerSec = motorSim.getAngularVelocityRPM() / 60.0;

    // Brake damping behavior
    if (Math.abs(appliedVolts) < 0.01) {
      double brakeDamping = 0.8;
      shooterVelocityRotPerSec *= brakeDamping;
      if (Math.abs(shooterVelocityRotPerSec) < 0.01) {
        shooterVelocityRotPerSec = 0.0;
      }
      motorSim.setState(motorSim.getAngularPositionRad(), shooterVelocityRotPerSec * TWO_PI);
    }

    rotorVelocityRotPerSec = shooterVelocityRotPerSec * kMotorToShooterGearRatio;

    currentAmps = Math.abs(motorSim.getCurrentDrawAmps());

    inputs.connected = true;
    inputs.rotorVelocity = RotationsPerSecond.of(rotorVelocityRotPerSec);
    inputs.shooterVelocity = RotationsPerSecond.of(shooterVelocityRotPerSec);
    inputs.requestedVelocity =
        Double.isFinite(targetVelocityRotPerSec)
            ? RotationsPerSecond.of(targetVelocityRotPerSec * kMotorToShooterGearRatio)
            : RotationsPerSecond.of(0.0);
    inputs.appliedVolts = appliedVolts;
    inputs.currentAmps = currentAmps;
  }

  @Override
  public void setOpenLoop(double volts) {
    targetVelocityRotPerSec = Double.NaN;
    appliedVolts = clamp(volts, -kMaxVoltage, kMaxVoltage);
  }

  @Override
  public void setVelocity(AngularVelocity velocity) {
    targetVelocityRotPerSec = velocity.in(RotationsPerSecond);
  }

  @Override
  public void zeroRotor() {
    motorSim.setState(0.0, 0.0);
    appliedVolts = 0.0;
    targetVelocityRotPerSec = Double.NaN;
  }
}
