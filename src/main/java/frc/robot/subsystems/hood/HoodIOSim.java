package frc.robot.subsystems.hood;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static frc.robot.subsystems.hood.HoodConstants.*;

import edu.wpi.first.math.MathUtil;
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
  private double targetAngleDeg = kInitialAngleOffsetDeg;
  private double appliedVolts = 0.0;

  public HoodIOSim() {
    motorSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(MOTOR_MODEL, HOOD_INERTIA, kMotorToHoodGearRatio),
            MOTOR_MODEL);
    positionController = new PIDController(kSimP, kSimI, kSimD);
    motorSim.setState(0.0, 0.0);
  }

  @Override
  public void updateInputs(HoodIOInputs inputs) {
    double motorRad = motorSim.getAngularPositionRad();
    double mechanismRad = motorRad / kMotorToHoodGearRatio;
    double hoodAngleDeg = kInitialAngleOffsetDeg - Math.toDegrees(mechanismRad);
    double hoodVelocityRadPerSec = -motorSim.getAngularVelocityRadPerSec() / kMotorToHoodGearRatio;

    if (positionControl) {
      double targetRad = Math.toRadians(targetAngleDeg);
      double currentRad = Math.toRadians(hoodAngleDeg);
      // Positive motor rotation decreases hood angle (65° -> 25°), so negate PID output
      double pidOutput = positionController.calculate(currentRad, targetRad);
      appliedVolts = MathUtil.clamp(-pidOutput, -kMaxVoltage, kMaxVoltage);
    }

    hoodAngleDeg = MathUtil.clamp(hoodAngleDeg, kMinAngle.in(Degrees), kMaxAngle.in(Degrees));
    // At 25° block positive voltage (would drive further down); at 65° block negative (would drive
    // up)
    if (hoodAngleDeg <= kMinAngle.in(Degrees) && appliedVolts > 0) appliedVolts = 0.0;
    if (hoodAngleDeg >= kMaxAngle.in(Degrees) && appliedVolts < 0) appliedVolts = 0.0;

    motorSim.setInputVoltage(appliedVolts);
    motorSim.update(SIMULATION_DT);

    motorRad = motorSim.getAngularPositionRad();
    mechanismRad = motorRad / kMotorToHoodGearRatio;
    hoodAngleDeg = kInitialAngleOffsetDeg - Math.toDegrees(mechanismRad);
    hoodVelocityRadPerSec = -motorSim.getAngularVelocityRadPerSec() / kMotorToHoodGearRatio;

    // Hard stop at limits
    double minDeg = kMinAngle.in(Degrees);
    double maxDeg = kMaxAngle.in(Degrees);
    if (hoodAngleDeg < minDeg) {
      motorSim.setState(
          (kInitialAngleOffsetDeg - minDeg) * (Math.PI / 180.0) * kMotorToHoodGearRatio, 0.0);
    } else if (hoodAngleDeg > maxDeg) {
      motorSim.setState(
          (kInitialAngleOffsetDeg - maxDeg) * (Math.PI / 180.0) * kMotorToHoodGearRatio, 0.0);
    }

    motorRad = motorSim.getAngularPositionRad();
    mechanismRad = motorRad / kMotorToHoodGearRatio;
    hoodAngleDeg = kInitialAngleOffsetDeg - Math.toDegrees(mechanismRad);
    hoodVelocityRadPerSec = -motorSim.getAngularVelocityRadPerSec() / kMotorToHoodGearRatio;

    inputs.connected = true;
    inputs.position = Degrees.of(hoodAngleDeg);
    inputs.velocity = RadiansPerSecond.of(hoodVelocityRadPerSec);
    inputs.appliedVolts = appliedVolts;
    inputs.currentAmps = Math.abs(motorSim.getCurrentDrawAmps());
  }

  @Override
  public void setOpenLoop(double volts) {
    positionControl = false;
    positionController.reset();
    appliedVolts = MathUtil.clamp(volts, -kMaxVoltage, kMaxVoltage);
  }

  @Override
  public void setPosition(Angle position) {
    positionControl = true;
    targetAngleDeg = position.in(Degrees);
    targetAngleDeg = MathUtil.clamp(targetAngleDeg, kMinAngle.in(Degrees), kMaxAngle.in(Degrees));
  }

  @Override
  public void zeroRotor() {
    motorSim.setState(0.0, 0.0);
    positionController.reset();
  }
}
