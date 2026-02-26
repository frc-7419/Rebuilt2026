package frc.robot.subsystems.serializer;

import static edu.wpi.first.math.MathUtil.*;
import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.serializer.SerializerConstants.*;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

/** Simulation implementation of SerializerIO: velocity control for serializer and feeder. */
public class SerializerIOSim implements SerializerIO {
  private static final double SIMULATION_DT = 0.02;
  private static final double SERIALIZER_INERTIA = 0.001;
  private static final DCMotor MOTOR_MODEL = DCMotor.getKrakenX44(1);

  private final DCMotorSim serializerMotorSim;
  private final DCMotorSim feederMotorSim;
  private final PIDController serializerVelocityController;
  private final PIDController feederVelocityController;

  private boolean serializerVelocityControl = false;
  private boolean feederVelocityControl = false;
  private AngularVelocity targetSerializerVelocity = RPM.of(0.0);
  private AngularVelocity targetFeederVelocity = RPM.of(0.0);
  private double serializerAppliedVolts = 0.0;
  private double feederAppliedVolts = 0.0;

  public SerializerIOSim() {
    serializerMotorSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                MOTOR_MODEL, SERIALIZER_INERTIA, kMotorToSerializerGearRatio),
            MOTOR_MODEL);
    feederMotorSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                MOTOR_MODEL, SERIALIZER_INERTIA, kMotorToFeederGearRatio),
            MOTOR_MODEL);
    serializerVelocityController = new PIDController(kSimP, kSimI, kSimD);
    feederVelocityController = new PIDController(kSimP, kSimI, kSimD);
  }

  @Override
  public void updateInputs(SerializerIOInputs inputs) {
    if (serializerVelocityControl) {
      double targetRadPerSec = targetSerializerVelocity.in(RadiansPerSecond);
      double currentRadPerSec = serializerMotorSim.getAngularVelocityRadPerSec();
      serializerAppliedVolts =
          clamp(
              serializerVelocityController.calculate(currentRadPerSec, targetRadPerSec),
              -12.0,
              12.0);
    }

    if (feederVelocityControl) {
      double targetRadPerSec = targetFeederVelocity.in(RadiansPerSecond);
      double currentRadPerSec = feederMotorSim.getAngularVelocityRadPerSec();
      feederAppliedVolts =
          clamp(feederVelocityController.calculate(currentRadPerSec, targetRadPerSec), -12.0, 12.0);
    }

    serializerMotorSim.setInputVoltage(serializerAppliedVolts);
    feederMotorSim.setInputVoltage(feederAppliedVolts);
    serializerMotorSim.update(SIMULATION_DT);
    feederMotorSim.update(SIMULATION_DT);

    inputs.serializerConnected = true;
    inputs.serializerAppliedVolts = serializerAppliedVolts;
    inputs.serializerCurrentAmps = Math.abs(serializerMotorSim.getCurrentDrawAmps());
    inputs.serializerVelocity = serializerMotorSim.getAngularVelocity();

    inputs.feederConnected = true;
    inputs.feederAppliedVolts = feederAppliedVolts;
    inputs.feederCurrentAmps = Math.abs(feederMotorSim.getCurrentDrawAmps());
    inputs.feederVelocity = feederMotorSim.getAngularVelocity();
  }

  @Override
  public void setSerializerOpenLoop(double volts) {
    serializerVelocityControl = false;
    serializerVelocityController.reset();
    serializerAppliedVolts = clamp(volts, -12.0, 12.0);
  }

  @Override
  public void setVelocity(AngularVelocity velocity) {
    serializerVelocityControl = true;
    targetSerializerVelocity = velocity;
  }

  @Override
  public void setFeederOpenLoop(double volts) {
    feederVelocityControl = false;
    feederVelocityController.reset();
    feederAppliedVolts = clamp(volts, -12.0, 12.0);
  }

  @Override
  public void setFeederVelocity(AngularVelocity velocity) {
    feederVelocityControl = true;
    targetFeederVelocity = velocity;
  }
}
