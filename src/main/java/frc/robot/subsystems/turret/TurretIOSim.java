package frc.robot.subsystems.turret;

import static frc.robot.subsystems.turret.TurretConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

/** Simulation implementation of TurretIO. */
public class TurretIOSim implements TurretIO {
  private static final double SIMULATION_DT = 0.02;
  private static final double TURRET_INERTIA = 0.1;
  private static final DCMotor MOTOR_MODEL = DCMotor.getKrakenX60Foc(1);

  // Motor simulation
  private final DCMotorSim motorSim;
  private final PIDController positionController;

  private boolean positionControl = false;
  private double targetPositionRad = 0.0;
  private double appliedVolts = 0.0;

  private double positionRad = 0.0;
  private double velocityRadPerSec = 0.0;
  private double currentAmps = 0.0;

  /** Creates a new TurretIOSim. */
  public TurretIOSim() {
    motorSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                MOTOR_MODEL, TURRET_INERTIA, kMotorToTurretGearRatio),
            MOTOR_MODEL);

    positionController = new PIDController(kP, kI, kD);
  }

  @Override
  public void updateInputs(TurretIOInputs inputs) {
    if (positionControl) {
      double pidOutput = positionController.calculate(positionRad, targetPositionRad);
      appliedVolts = MathUtil.clamp(pidOutput, -kMaxVoltage, kMaxVoltage);
    }

    if (positionRad <= kMinAngleRad && appliedVolts < 0) {
      appliedVolts = 0.0;
      motorSim.setState(kMinAngleRad, 0.0);
    } else if (positionRad >= kMaxAngleRad && appliedVolts > 0) {
      appliedVolts = 0.0;
      motorSim.setState(kMaxAngleRad, 0.0);
    }

    if (positionRad < kMinAngleRad) {
      positionRad = kMinAngleRad;
      motorSim.setState(positionRad, 0.0);
    } else if (positionRad > kMaxAngleRad) {
      positionRad = kMaxAngleRad;
      motorSim.setState(positionRad, 0.0);
    }

    motorSim.setInputVoltage(appliedVolts);
    motorSim.update(SIMULATION_DT);

    positionRad = motorSim.getAngularPositionRad();
    velocityRadPerSec = motorSim.getAngularVelocityRadPerSec();

    if (Math.abs(appliedVolts) < 0.01) {
      double brakeDamping = 0.8;
      velocityRadPerSec *= brakeDamping;
      if (Math.abs(velocityRadPerSec) < 0.01) {
        velocityRadPerSec = 0.0;
      }
      motorSim.setState(positionRad, velocityRadPerSec);
    }

    currentAmps = Math.abs(motorSim.getCurrentDrawAmps());

    inputs.connected = true;
    inputs.position = new Rotation2d(positionRad);
    inputs.absolutePosition = inputs.position;
    inputs.velocityRadPerSec = velocityRadPerSec;
    inputs.appliedVolts = appliedVolts;
    inputs.currentAmps = currentAmps;
  }

  @Override
  public void setOpenLoop(double volts) {
    positionControl = false;
    positionController.reset();

    appliedVolts = MathUtil.clamp(volts, -kMaxVoltage, kMaxVoltage);
  }

  @Override
  public void setPosition(Rotation2d position) {
    positionControl = true;

    double targetRad = position.getRadians();
    targetRad = MathUtil.clamp(targetRad, kMinAngleRad, kMaxAngleRad);
    targetPositionRad = targetRad;
  }
}
