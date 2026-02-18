package frc.robot.subsystems.intake;

import static edu.wpi.first.math.MathUtil.*;
import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.intake.IntakeConstants.*;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

/** Simulation implementation of IntakeIO: position control for wrist, velocity control for wheel. */
public class IntakeIOSim implements IntakeIO {
  private static final double SIMULATION_DT = 0.02;

  private static final double WHEEL_INERTIA = 0.001;
  private static final double WRIST_INERTIA = 0.05;
  private static final DCMotor WHEEL_MOTOR = DCMotor.getKrakenX60(1);
  private static final DCMotor WRIST_MOTOR = DCMotor.getKrakenX44(1);

  private final DCMotorSim wheelMotorSim;
  private final DCMotorSim wristMotorSim;
  private final PIDController wheelVelocityController;
  private final PIDController wristPositionController;

  private boolean wheelVelocityControl = false;
  private boolean wristPositionControl = false;
  private AngularVelocity targetWheelVelocity = RPM.of(0.0);
  private Angle targetWristAngle = kMinWristAngle;
  private double wheelAppliedVolts = 0.0;
  private double wristAppliedVolts = 0.0;

  public IntakeIOSim() {
    wheelMotorSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(WHEEL_MOTOR, WHEEL_INERTIA, kWheelMotorToWheelGearRatio),
            WHEEL_MOTOR);
    wristMotorSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(WRIST_MOTOR, WRIST_INERTIA, kWristMotorToWristGearRatio),
            WRIST_MOTOR);
    wheelVelocityController = new PIDController(kSimP, kSimI, kSimD);
    wristPositionController = new PIDController(kSimP, kSimI, kSimD);
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    // Wheel velocity PID
    if (wheelVelocityControl) {
      double targetRadPerSec = targetWheelVelocity.in(RadiansPerSecond);
      double currentRadPerSec = wheelMotorSim.getAngularVelocityRadPerSec();
      wheelAppliedVolts =
          clamp(
              wheelVelocityController.calculate(currentRadPerSec, targetRadPerSec),
              -kMaxVoltage,
              kMaxVoltage);
    }

    // Wrist position PID
    double wristRad = wristMotorSim.getAngularPositionRad();
    if (wristPositionControl) {
      double targetRad = targetWristAngle.in(Radians);
      wristAppliedVolts =
          clamp(
              wristPositionController.calculate(wristRad, targetRad),
              -kMaxVoltage,
              kMaxVoltage);
    }

    // Wrist boundary enforcement
    wristRad = clamp(wristRad, kMinWristAngle.in(Radians), kMaxWristAngle.in(Radians));
    if (wristRad <= kMinWristAngle.in(Radians) && wristAppliedVolts < 0) {
      wristAppliedVolts = 0.0;
      wristMotorSim.setState(kMinWristAngle.in(Radians), 0.0);
    }
    if (wristRad >= kMaxWristAngle.in(Radians) && wristAppliedVolts > 0) {
      wristAppliedVolts = 0.0;
      wristMotorSim.setState(kMaxWristAngle.in(Radians), 0.0);
    }

    wheelMotorSim.setInputVoltage(wheelAppliedVolts);
    wristMotorSim.setInputVoltage(wristAppliedVolts);
    wheelMotorSim.update(SIMULATION_DT);
    wristMotorSim.update(SIMULATION_DT);

    inputs.wheelConnected = true;
    inputs.wheelAppliedVolts = wheelAppliedVolts;
    inputs.wheelCurrentAmps = Math.abs(wheelMotorSim.getCurrentDrawAmps());
    inputs.wheelVelocity = wheelMotorSim.getAngularVelocity();

    inputs.wristConnected = true;
    inputs.wristAppliedVolts = wristAppliedVolts;
    inputs.wristCurrentAmps = Math.abs(wristMotorSim.getCurrentDrawAmps());
    inputs.wristPosition = wristMotorSim.getAngularPosition();
    inputs.wristVelocity = wristMotorSim.getAngularVelocity();
  }

  @Override
  public void setWheelOpenLoop(double volts) {
    wheelVelocityControl = false;
    wheelVelocityController.reset();
    wheelAppliedVolts = clamp(volts, -kMaxVoltage, kMaxVoltage);
  }

  @Override
  public void setWheelVelocity(AngularVelocity velocity) {
    wheelVelocityControl = true;
    targetWheelVelocity = velocity;
  }

  @Override
  public void setWristOpenLoop(double volts) {
    wristPositionControl = false;
    wristPositionController.reset();
    wristAppliedVolts = clamp(volts, -kMaxVoltage, kMaxVoltage);
  }

  @Override
  public void setWristPosition(Angle angle) {
    wristPositionControl = true;
    double deg = clamp(angle.in(Degrees), kMinWristAngle.in(Degrees), kMaxWristAngle.in(Degrees));
    targetWristAngle = Degrees.of(deg);
  }

  @Override
  public void zeroWrist() {
    wristMotorSim.setState(0.0, 0.0);
    wristPositionController.reset();
  }
}
