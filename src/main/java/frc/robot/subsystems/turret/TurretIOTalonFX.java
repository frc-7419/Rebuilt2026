package frc.robot.subsystems.turret;

import static edu.wpi.first.units.Units.Rotations;
import static frc.robot.subsystems.turret.TurretConstants.kMotorToTurretGearRatio;
import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

/** TalonFX implementation of TurretIO. */
public class TurretIOTalonFX implements TurretIO {
  private final TalonFX motor;

  private final StatusSignal<Voltage> motorAppliedVolts;
  private final StatusSignal<Current> motorCurrent;
  private final StatusSignal<AngularVelocity> motorVelocity;
  private final StatusSignal<Angle> motorPosition;

  // Control requests
  private final VoltageOut voltageRequest = new VoltageOut(0);
  private final PositionVoltage positionVoltageRequest = new PositionVoltage(0.0);

  public TurretIOTalonFX() {
    motor = new TalonFX(TurretConstants.kTurretMotorId);
    tryUntilOk(5, () -> motor.getConfigurator().apply(TurretConstants.motorConfig, 0.25));

    motorAppliedVolts = motor.getMotorVoltage();
    motorCurrent = motor.getStatorCurrent();
    motorVelocity = motor.getVelocity();
    motorPosition = motor.getPosition();

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0, motorAppliedVolts, motorCurrent, motorVelocity, motorPosition);
  }

  @Override
  public void updateInputs(TurretIOInputs inputs) {
    var status =
        BaseStatusSignal.refreshAll(motorAppliedVolts, motorCurrent, motorVelocity, motorPosition);
    inputs.connected = status.equals(StatusCode.OK);
    inputs.appliedVolts = motorAppliedVolts.getValueAsDouble();
    inputs.currentAmps = motorCurrent.getValueAsDouble();
    inputs.velocity = motorVelocity.getValue().div(kMotorToTurretGearRatio);
    inputs.rotorPosition = motorPosition.getValue();
    inputs.turretPosition = motorPosition.getValue().div(kMotorToTurretGearRatio);
  }

  @Override
  public void setOpenLoop(double volts) {
    motor.setControl(voltageRequest.withOutput(volts));
  }

  @Override
  public void setPosition(Angle position) {
    motor.setControl(positionVoltageRequest.withPosition(position.in(Rotations)));
  }

  @Override
  public void setState(Angle position, AngularVelocity velocity) {
    motor.setControl(
        positionVoltageRequest.withPosition(position.in(Rotations)).withVelocity(velocity));
  }

  @Override
  public void zeroRotor(Angle offset) {
    motor.setPosition(offset);
  }
}
