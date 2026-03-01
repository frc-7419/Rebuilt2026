package frc.robot.subsystems.serializer;

import static frc.robot.subsystems.serializer.SerializerConstants.kMotorToFeederGearRatio;
import static frc.robot.subsystems.serializer.SerializerConstants.kMotorToSerializerGearRatio;
import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

public class SerializerIOTalonFX implements SerializerIO {
  private final TalonFX serializerMotor;
  private final TalonFX feederMotor;

  private final StatusSignal<Voltage> serializerAppliedVolts;
  private final StatusSignal<Current> serializerCurrent;
  private final StatusSignal<AngularVelocity> serializerVelocity;

  private final StatusSignal<Voltage> feederAppliedVolts;
  private final StatusSignal<Current> feederCurrent;
  private final StatusSignal<AngularVelocity> feederVelocity;

  private final VoltageOut voltageRequest = new VoltageOut(0);
  private final VelocityVoltage velocityVoltageRequest = new VelocityVoltage(0.0);
  private final VoltageOut feederVoltageRequest = new VoltageOut(0);
  private final VelocityVoltage feederVelocityVoltageRequest = new VelocityVoltage(0.0);

  public SerializerIOTalonFX() {
    serializerMotor = new TalonFX(SerializerConstants.kSerializerMotorId);
    feederMotor = new TalonFX(SerializerConstants.kFeederMotorId);

    tryUntilOk(
        5,
        () ->
            serializerMotor
                .getConfigurator()
                .apply(SerializerConstants.serializerMotorConfig, 0.25));
    tryUntilOk(
        5, () -> feederMotor.getConfigurator().apply(SerializerConstants.feederMotorConfig, 0.25));

    serializerAppliedVolts = serializerMotor.getMotorVoltage();
    serializerCurrent = serializerMotor.getStatorCurrent();
    serializerVelocity = serializerMotor.getVelocity();

    feederAppliedVolts = feederMotor.getMotorVoltage();
    feederCurrent = feederMotor.getStatorCurrent();
    feederVelocity = feederMotor.getVelocity();

    BaseStatusSignal.setUpdateFrequencyForAll(
        4.0,
        serializerAppliedVolts,
        serializerCurrent,
        serializerVelocity,
        feederAppliedVolts,
        feederCurrent,
        feederVelocity);
  }

  @Override
  public void updateInputs(SerializerIOInputs inputs) {
    var status =
        BaseStatusSignal.refreshAll(
            serializerAppliedVolts,
            serializerCurrent,
            serializerVelocity,
            feederAppliedVolts,
            feederCurrent,
            feederVelocity);
    inputs.serializerConnected = status.equals(StatusCode.OK);
    inputs.feederConnected = status.equals(StatusCode.OK);

    inputs.serializerAppliedVolts = serializerAppliedVolts.getValueAsDouble();
    inputs.serializerCurrentAmps = serializerCurrent.getValueAsDouble();
    inputs.serializerVelocity = serializerVelocity.getValue().div(kMotorToSerializerGearRatio);

    inputs.feederAppliedVolts = feederAppliedVolts.getValueAsDouble();
    inputs.feederCurrentAmps = feederCurrent.getValueAsDouble();
    inputs.feederVelocity = feederVelocity.getValue().div(kMotorToFeederGearRatio);
  }

  @Override
  public void setSerializerOpenLoop(double volts) {
    serializerMotor.setControl(voltageRequest.withOutput(volts));
  }

  @Override
  public void setVelocity(AngularVelocity velocity) {
    serializerMotor.setControl(
        velocityVoltageRequest.withVelocity(velocity.times(kMotorToSerializerGearRatio)));
  }

  @Override
  public void setFeederOpenLoop(double volts) {
    feederMotor.setControl(feederVoltageRequest.withOutput(volts));
  }

  @Override
  public void setFeederVelocity(AngularVelocity velocity) {
    feederMotor.setControl(
        feederVelocityVoltageRequest.withVelocity(velocity.times(kMotorToFeederGearRatio)));
  }
}
