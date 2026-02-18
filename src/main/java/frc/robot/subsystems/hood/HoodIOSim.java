package frc.robot.subsystems.hood;

import static edu.wpi.first.math.MathUtil.*;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;
import static frc.robot.subsystems.hood.HoodConstants.*;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

/** Simulation implementation of HoodIO: position control and open loop. */
public class HoodIOSim implements HoodIO {
  private static final double SIMULATION_DT = 0.02;
  private static final double HOOD_INERTIA = 0.05;
  private static final DCMotor MOTOR_MODEL = DCMotor.getKrakenX60Foc(1);

  private final DCMotorSim motorSim;
  private final PIDController positionController;

  private boolean positionControl = false;
  private Angle targetAngle = kMaxAngle;
  private double appliedVolts = 0.0;

  public HoodIOSim() {
    motorSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(MOTOR_MODEL, HOOD_INERTIA, kMotorToHoodGearRatio),
            MOTOR_MODEL);
    positionController = new PIDController(kSimP, kSimI, kSimD);
    motorSim.setState(kMaxAngle.in(Radians), 0.0);
  }

  @Override
  public void updateInputs(HoodIOInputs inputs) {
    double mechanismRad = motorSim.getAngularPositionRad();
    double rotorRad = mechanismRad * kMotorToHoodGearRatio;

    if (positionControl) {
      double targetRad = targetAngle.in(Radians);
      double currentRad = mechanismRad;

      double pidOutput = positionController.calculate(currentRad, targetRad);
      appliedVolts = clamp(-pidOutput, -kMaxVoltage, kMaxVoltage);
    }

    mechanismRad = clamp(mechanismRad, kMinAngle.in(Radians), kMaxAngle.in(Radians));

    if (mechanismRad <= kMinAngle.in(Radians) && appliedVolts < 0) {
      appliedVolts = 0.0;
      motorSim.setState(kMinAngle.in(Radians), 0.0);
    }
    if (mechanismRad >= kMaxAngle.in(Radians) && appliedVolts > 0) {
      appliedVolts = 0.0;
      motorSim.setState(kMaxAngle.in(Radians), 0.0);
    }

    motorSim.setInputVoltage(appliedVolts);
    motorSim.update(SIMULATION_DT);

    rotorRad = motorSim.getAngularPositionRad();
    mechanismRad = rotorRad / kMotorToHoodGearRatio;

    inputs.connected = true;
    inputs.position = motorSim.getAngularPosition();
    inputs.rotorPosition = Radians.of(rotorRad);
    inputs.velocity = motorSim.getAngularVelocity();
    inputs.appliedVolts = appliedVolts;
    inputs.currentAmps = Math.abs(motorSim.getCurrentDrawAmps());
  }

  @Override
  public void setOpenLoop(double volts) {
    positionControl = false;
    positionController.reset();
    appliedVolts = clamp(volts, -kMaxVoltage, kMaxVoltage);
  }

  @Override
  public void setPosition(Angle position) {
    positionControl = true;
    double positionDeg = clamp(position.in(Degrees), kMinAngle.in(Degrees), kMaxAngle.in(Degrees));
    targetAngle = Degrees.of(positionDeg);
  }

  @Override
  public void zeroRotor() {
    motorSim.setState(0.0, 0.0);
    positionController.reset();
  }
}
