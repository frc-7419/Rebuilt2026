package frc.robot.subsystems.serializer;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

/** Simulation implementation of SerializerIO. */
public class SerializerIOSim implements SerializerIO {
  private static final double LOOP_PERIOD_SECS = 0.02;

  private static final double SERIALIZER_INERTIA = 0.001;
  private static final DCMotor MOTOR_MODEL = DCMotor.getFalcon500(1);

  private final DCMotorSim serializerMotorSim;
  private final DCMotorSim feederMotorSim;

  private double serializerAppliedVolts = 0.0;
  private double feederAppliedVolts = 0.0;

  public SerializerIOSim() {
    serializerMotorSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                MOTOR_MODEL, SERIALIZER_INERTIA, SerializerConstants.kMotorToSerializerGearRatio),
            MOTOR_MODEL);
    feederMotorSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                MOTOR_MODEL, SERIALIZER_INERTIA, SerializerConstants.kMotorToFeederGearRatio),
            MOTOR_MODEL);
  }

  @Override
  public void updateInputs(SerializerIOInputs inputs) {
    serializerMotorSim.update(LOOP_PERIOD_SECS);
    feederMotorSim.update(LOOP_PERIOD_SECS);

    inputs.serializerConnected = true;
    inputs.serializerAppliedVolts = serializerAppliedVolts;
    inputs.serializerCurrentAmps = serializerMotorSim.getCurrentDrawAmps();
    inputs.serializerVelocity =
        edu.wpi.first.units.Units.RPM.of(serializerMotorSim.getAngularVelocityRPM());

    inputs.feederConnected = true;
    inputs.feederAppliedVolts = feederAppliedVolts;
    inputs.feederCurrentAmps = feederMotorSim.getCurrentDrawAmps();
    inputs.feederVelocity =
        edu.wpi.first.units.Units.RPM.of(feederMotorSim.getAngularVelocityRPM());
  }

  @Override
  public void setOpenLoop(double volts) {
    serializerAppliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
    serializerMotorSim.setInputVoltage(serializerAppliedVolts);
  }

  @Override
  public void setVelocity(AngularVelocity velocity) {
    // Simple proportional control for simulation
    double targetRPM = velocity.in(edu.wpi.first.units.Units.RPM);
    double currentRPM = serializerMotorSim.getAngularVelocityRPM();
    double error = targetRPM - currentRPM;
    double volts = error * SerializerConstants.kSimP;
    setOpenLoop(volts);
  }

  @Override
  public void setFeederOpenLoop(double volts) {
    feederAppliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
    feederMotorSim.setInputVoltage(feederAppliedVolts);
  }

  @Override
  public void setFeederVelocity(AngularVelocity velocity) {
    // Simple proportional control for simulation
    double targetRPM = velocity.in(edu.wpi.first.units.Units.RPM);
    double currentRPM = feederMotorSim.getAngularVelocityRPM();
    double error = targetRPM - currentRPM;
    double volts = error * SerializerConstants.kSimP;
    setFeederOpenLoop(volts);
  }
}
