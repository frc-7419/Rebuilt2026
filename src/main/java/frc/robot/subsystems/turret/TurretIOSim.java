package frc.robot.subsystems.turret;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static frc.robot.subsystems.turret.TurretConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

/** Simulation implementation of TurretIO. */
public class TurretIOSim implements TurretIO {
  private static final double SIMULATION_DT = 0.02;
  private static final double TURRET_INERTIA = 0.1;
  private static final DCMotor MOTOR_MODEL = DCMotor.getKrakenX60Foc(1);
  private static final double kV = 12.0 / MOTOR_MODEL.freeSpeedRadPerSec; // Velocity feedforward

  // Motor simulation
  private final DCMotorSim motorSim;
  private final PIDController positionController;

  private boolean positionControl = false;
  private double targetPositionRad = 0.0;
  private double feedforwardVelocityRadPerSec = 0.0;
  private double appliedVolts = 0.0;

  private double rotorPositionRad = 0.0;
  private double turretPositionRad = 0.0;
  private double velocityRadPerSec = 0.0;
  private double currentAmps = 0.0;

  private double encoderOnePosition = 0.0;
  private double encoderTwoPosition = 0.0;

  /** Creates a new TurretIOSim. */
  public TurretIOSim() {
    motorSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                MOTOR_MODEL, TURRET_INERTIA, kMotorToTurretGearRatio),
            MOTOR_MODEL);

    positionController = new PIDController(kSimP, kSimI, kSimD);
  }

  @Override
  public void updateInputs(TurretIOInputs inputs) {
    if (positionControl) {
      double pidOutput = positionController.calculate(turretPositionRad, targetPositionRad);
      double feedforwardVolts = feedforwardVelocityRadPerSec * kV;
      appliedVolts = MathUtil.clamp(pidOutput + feedforwardVolts, -kMaxVoltage, kMaxVoltage);
    }

    if (turretPositionRad <= kMinAngle.in(Radians) && appliedVolts < 0) {
      appliedVolts = 0.0;
      motorSim.setState(kMinAngle.in(Radians), 0.0);
    } else if (turretPositionRad >= kMaxAngle.in(Radians) && appliedVolts > 0) {
      appliedVolts = 0.0;
      motorSim.setState(kMaxAngle.in(Radians), 0.0);
    }

    if (turretPositionRad < kMinAngle.in(Radians)) {
      turretPositionRad = kMinAngle.in(Radians);
      motorSim.setState(turretPositionRad, 0.0);
    } else if (turretPositionRad > kMaxAngle.in(Radians)) {
      turretPositionRad = kMaxAngle.in(Radians);
      motorSim.setState(turretPositionRad, 0.0);
    }

    motorSim.setInputVoltage(appliedVolts);
    motorSim.update(SIMULATION_DT);

    turretPositionRad = motorSim.getAngularPositionRad();
    rotorPositionRad = turretPositionRad * kMotorToTurretGearRatio;
    velocityRadPerSec = motorSim.getAngularVelocityRadPerSec();

    if (Math.abs(appliedVolts) < 0.01) {
      double brakeDamping = 0.8;
      velocityRadPerSec *= brakeDamping;
      if (Math.abs(velocityRadPerSec) < 0.01) {
        velocityRadPerSec = 0.0;
      }
      motorSim.setState(turretPositionRad, velocityRadPerSec);
    }

    currentAmps = Math.abs(motorSim.getCurrentDrawAmps());

    encoderOnePosition =
        ((rotorPositionRad * kMotorToEncoderOneGearRatio) + encoderOneZeroOffset.in(Radians))
            % (2.0 * Math.PI);
    encoderTwoPosition =
        ((rotorPositionRad * kMotorToEncoderTwoGearRatio) + encoderTwoZeroOffset.in(Radians))
            % (2.0 * Math.PI);

    inputs.connected = true;
    inputs.rotorPosition = Radians.of(rotorPositionRad);
    inputs.turretPosition = Radians.of(turretPositionRad);
    inputs.velocity = RadiansPerSecond.of(velocityRadPerSec);
    inputs.appliedVolts = appliedVolts;
    inputs.currentAmps = currentAmps;
    inputs.encoderOnePosition = Radians.of(encoderOnePosition);
    inputs.encoderTwoPosition = Radians.of(encoderTwoPosition);
  }

  @Override
  public void setOpenLoop(double volts) {
    positionControl = false;
    positionController.reset();
    feedforwardVelocityRadPerSec = 0.0;

    appliedVolts = MathUtil.clamp(volts, -kMaxVoltage, kMaxVoltage);
  }

  @Override
  public void setPosition(Angle position) {
    positionControl = true;
    feedforwardVelocityRadPerSec = 0.0;

    double targetRad = position.in(Radians);
    targetRad = MathUtil.clamp(targetRad, kMinAngle.in(Radians), kMaxAngle.in(Radians));
    targetPositionRad = targetRad;
  }

  @Override
  public void setState(Angle position, AngularVelocity velocity) {
    positionControl = true;

    double targetRad = position.in(Radians);
    targetRad = MathUtil.clamp(targetRad, kMinAngle.in(Radians), kMaxAngle.in(Radians));
    targetPositionRad = targetRad;
    feedforwardVelocityRadPerSec = velocity.in(RadiansPerSecond);
  }
}
