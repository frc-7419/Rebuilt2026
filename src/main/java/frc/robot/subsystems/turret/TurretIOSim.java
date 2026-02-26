package frc.robot.subsystems.turret;

import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static frc.robot.subsystems.turret.TurretConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

/** Simulation implementation of TurretIO. */
public class TurretIOSim implements TurretIO {
  private static final double SIMULATION_DT = 0.02;
  private static final double TURRET_INERTIA = 0.1;
  private static final DCMotor MOTOR_MODEL = DCMotor.getKrakenX60Foc(1);
  private static final double kV =
      12.0 / (MOTOR_MODEL.freeSpeedRadPerSec / kMotorToTurretGearRatio); // Velocity feedforward

  // Motor simulation
  private final DCMotorSim motorSim;
  private final PIDController positionController;

  private boolean positionControl = false;
  private TrapezoidProfile.State goalState;
  private double targetPositionRad = 0.0;
  private double feedforwardVelocityRadPerSec = 0.0;
  private double appliedVolts = 0.0;

  private double rotorPositionRad = 0.0;
  private double turretPositionRad = 0.0;
  private double velocityRadPerSec = 0.0;
  private double currentAmps = 0.0;

  private double rightEncoderPosition = 0.0;
  private double leftEncoderPosition = 0.0;

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
    if (positionControl && goalState != null) {
      TrapezoidProfile.State currentState =
          new TrapezoidProfile.State(
              turretPositionRad / (2.0 * Math.PI), velocityRadPerSec / (2.0 * Math.PI));
      TrapezoidProfile.State setpoint =
          TurretConstants.kMotionProfile.calculate(SIMULATION_DT, currentState, goalState);
      targetPositionRad =
          MathUtil.clamp(
              setpoint.position * (2.0 * Math.PI),
              kAbsoluteMinAngle.in(Radians),
              kAbsoluteMaxAngle.in(Radians));
      feedforwardVelocityRadPerSec = setpoint.velocity * (2.0 * Math.PI);
      inputs.requestedPosition = Radians.of(targetPositionRad);
      inputs.requestedVelocity = RadiansPerSecond.of(feedforwardVelocityRadPerSec);
    }

    if (positionControl) {
      double pidOutput = positionController.calculate(turretPositionRad, targetPositionRad);
      double feedforwardVolts = feedforwardVelocityRadPerSec * kV;
      appliedVolts = MathUtil.clamp(pidOutput + feedforwardVolts, -kMaxVoltage, kMaxVoltage);
    }

    if (turretPositionRad <= kAbsoluteMinAngle.in(Radians) && appliedVolts < 0) {
      appliedVolts = 0.0;
      motorSim.setState(kAbsoluteMinAngle.in(Radians), 0.0);
    } else if (turretPositionRad >= kAbsoluteMaxAngle.in(Radians) && appliedVolts > 0) {
      appliedVolts = 0.0;
      motorSim.setState(kAbsoluteMaxAngle.in(Radians), 0.0);
    }

    if (turretPositionRad < kAbsoluteMinAngle.in(Radians)) {
      turretPositionRad = kAbsoluteMinAngle.in(Radians);
      motorSim.setState(turretPositionRad, 0.0);
    } else if (turretPositionRad > kAbsoluteMaxAngle.in(Radians)) {
      turretPositionRad = kAbsoluteMaxAngle.in(Radians);
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

    rightEncoderPosition =
        ((rotorPositionRad * kMotorToRightEncoderGearRatio) + rightEncoderZeroOffset.in(Radians))
            % (2.0 * Math.PI);
    leftEncoderPosition =
        ((rotorPositionRad * kMotorToLeftEncoderGearRatio) + leftEncoderZeroOffset.in(Radians))
            % (2.0 * Math.PI);

    inputs.connected = true;
    inputs.rotorPosition = Radians.of(rotorPositionRad);
    inputs.turretPosition = Radians.of(turretPositionRad);
    inputs.velocity = RadiansPerSecond.of(velocityRadPerSec);
    inputs.appliedVolts = appliedVolts;
    inputs.currentAmps = currentAmps;
    inputs.rightEncoderPosition = Radians.of(rightEncoderPosition);
    inputs.leftEncoderPosition = Radians.of(leftEncoderPosition);
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
    setState(position, RPM.of(0));
  }

  @Override
  public void setState(Angle position, AngularVelocity velocity) {
    positionControl = true;
    goalState = new TrapezoidProfile.State(position.in(Rotations), velocity.in(RotationsPerSecond));
    // Setpoint is computed every cycle in updateInputs from current state + goal
  }
}
