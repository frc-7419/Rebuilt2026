package frc.robot.subsystems.hopper;

import static frc.robot.subsystems.hopper.HopperConstants.kMotorToHopperGearRatio;
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

public class HopperIOTalonFX implements HopperIO {
  private final TalonFX motor;

  private final StatusSignal<Voltage> motorAppliedVolts;
  private final StatusSignal<Current> motorCurrent;
  private final StatusSignal<AngularVelocity> motorVelocity;

  private final VoltageOut voltageRequest = new VoltageOut(0);
  private final VelocityVoltage velocityVoltageRequest = new VelocityVoltage(0.0);

  public HopperIOTalonFX() {
    motor = new TalonFX(HopperConstants.kHopperMotorId);

    tryUntilOk(5, () -> motor.getConfigurator().apply(HopperConstants.motorConfig, 0.25));

    motorAppliedVolts = motor.getMotorVoltage();
    motorCurrent = motor.getStatorCurrent();
    motorVelocity = motor.getVelocity();

    BaseStatusSignal.setUpdateFrequencyForAll(50.0, motorAppliedVolts, motorCurrent, motorVelocity);
  }

  @Override
  public void updateInputs(HopperIOInputs inputs) {
    var status = BaseStatusSignal.refreshAll(motorAppliedVolts, motorCurrent, motorVelocity);
    inputs.connected = status.equals(StatusCode.OK);

    inputs.appliedVolts = motorAppliedVolts.getValueAsDouble();
    inputs.currentAmps = motorCurrent.getValueAsDouble();
    inputs.velocity = motorVelocity.getValue().div(kMotorToHopperGearRatio);
  }

  @Override
  public void setOpenLoop(double volts) {
    motor.setControl(voltageRequest.withOutput(volts));
  }

  @Override
  public void setVelocity(AngularVelocity velocity) {
    motor.setControl(velocityVoltageRequest.withVelocity(velocity.div(kMotorToHopperGearRatio)));
  }
}
