package frc.robot.subsystems.hood;

import static frc.robot.subsystems.hood.HoodConstants.kMaxAngle;
import static frc.robot.subsystems.hood.HoodConstants.kMotorToHoodGearRatio;
import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.Logger;

/** TalonFX implementation of HoodIO: position (MotionMagic) and open loop. */
public class HoodIOTalonFX implements HoodIO {
  private final TalonFX motor;

  private final StatusSignal<Voltage> motorAppliedVolts;
  private final StatusSignal<Current> motorCurrent;
  private final StatusSignal<AngularVelocity> motorVelocity;
  private final StatusSignal<Angle> motorPosition;

  private final VoltageOut voltageRequest = new VoltageOut(0);
  private final MotionMagicVoltage motionMagicRequest = new MotionMagicVoltage(0.0);

  public HoodIOTalonFX() {
    motor = new TalonFX(HoodConstants.kHoodMotorId);
    tryUntilOk(5, () -> motor.getConfigurator().apply(HoodConstants.motorConfig, 0.25));

    motorAppliedVolts = motor.getMotorVoltage();
    motorCurrent = motor.getStatorCurrent();
    motorVelocity = motor.getVelocity();
    motorPosition = motor.getPosition();

    zeroRotor();

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0, motorAppliedVolts, motorCurrent, motorVelocity, motorPosition);
  }

  @Override
  public void updateInputs(HoodIOInputs inputs) {
    var status =
        BaseStatusSignal.refreshAll(motorAppliedVolts, motorCurrent, motorVelocity, motorPosition);
    inputs.connected = status.equals(StatusCode.OK);
    inputs.appliedVolts = motorAppliedVolts.getValueAsDouble();
    inputs.currentAmps = motorCurrent.getValueAsDouble();

    inputs.position = kMaxAngle.minus(motorPosition.getValue().div(kMotorToHoodGearRatio));
    inputs.rotorPosition = motorPosition.getValue();

    inputs.velocity = motorVelocity.getValue().div(kMotorToHoodGearRatio);
  }

  @Override
  public void setOpenLoop(double volts) {
    motor.setControl(voltageRequest.withOutput(volts));
  }

  @Override
  public void setPosition(Angle position) {
    motor.setControl(
        motionMagicRequest.withPosition(position.in(Units.Rotations) * (kMotorToHoodGearRatio)));
    Logger.recordOutput(
        "Hood/RequestedPositionRotations", position.in(Units.Rotations) * (kMotorToHoodGearRatio));
  }

  @Override
  public void zeroRotor() {
    motor.setPosition(0.0);
  }
}
