package frc.robot.subsystems.turret;

import static edu.wpi.first.units.Units.Rotations;
import static frc.robot.subsystems.turret.TurretConstants.kMotorToTurretGearRatio;
import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.generated.TunerConstants;

/** TalonFX implementation of TurretIO. */
public class TurretIOTalonFX implements TurretIO {
  private final TalonFX motor;
  private final CANcoder rightEncoder;
  private final CANcoder leftEncoder;

  private final StatusSignal<Voltage> motorAppliedVolts;
  private final StatusSignal<Current> motorCurrent;
  private final StatusSignal<AngularVelocity> motorVelocity;
  private final StatusSignal<Angle> motorPosition;
  private final StatusSignal<Angle> rightEncoderPosition;
  private final StatusSignal<Angle> leftEncoderPosition;

  // Control requests
  private final VoltageOut voltageRequest = new VoltageOut(0);
  private final PositionVoltage positionVoltageRequest = new PositionVoltage(0.0);

  public TurretIOTalonFX() {
    motor = new TalonFX(TurretConstants.kTurretMotorId);
    rightEncoder = new CANcoder(TurretConstants.kEncoderRightId, TunerConstants.kCANBus);
    leftEncoder = new CANcoder(TurretConstants.kEncoderLeftId, TunerConstants.kCANBus);

    tryUntilOk(5, () -> motor.getConfigurator().apply(TurretConstants.motorConfig, 0.25));
    tryUntilOk(5, () -> rightEncoder.getConfigurator().apply(TurretConstants.cancoderConfig, 0.25));
    tryUntilOk(5, () -> leftEncoder.getConfigurator().apply(TurretConstants.cancoderConfig, 0.25));

    motorAppliedVolts = motor.getMotorVoltage();
    motorCurrent = motor.getStatorCurrent();
    motorVelocity = motor.getVelocity();
    motorPosition = motor.getPosition();

    rightEncoderPosition = rightEncoder.getAbsolutePosition();
    leftEncoderPosition = leftEncoder.getAbsolutePosition();

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0, motorAppliedVolts, motorCurrent, motorVelocity, motorPosition);

    BaseStatusSignal.setUpdateFrequencyForAll(200.0, rightEncoderPosition, leftEncoderPosition);
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
    inputs.turretPosition = motorPosition.getValue();
    inputs.rightEncoderPosition = rightEncoderPosition.getValue();
    inputs.leftEncoderPosition = leftEncoderPosition.getValue();
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
