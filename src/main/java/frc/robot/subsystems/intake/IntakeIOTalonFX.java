package frc.robot.subsystems.intake;

import static frc.robot.subsystems.intake.IntakeConstants.kWheelMotorToWheelGearRatio;
import static frc.robot.subsystems.intake.IntakeConstants.kWristMotorToWristGearRatio;
import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

public class IntakeIOTalonFX implements IntakeIO {
  private final TalonFX wheelMotor;
  private final TalonFX wristMotor;

  // Wheel motor signals
  private final StatusSignal<Voltage> wheelAppliedVolts;
  private final StatusSignal<Current> wheelCurrent;
  private final StatusSignal<AngularVelocity> wheelVelocity;

  // Wrist motor signals
  private final StatusSignal<Voltage> wristAppliedVolts;
  private final StatusSignal<Current> wristCurrent;
  private final StatusSignal<Angle> wristPosition;
  private final StatusSignal<AngularVelocity> wristVelocity;

  private final VoltageOut wheelVoltageRequest = new VoltageOut(0);
  private final VelocityVoltage wheelVelocityRequest = new VelocityVoltage(0.0);
  private final MotionMagicVelocityVoltage wheelMotionMagicVelocityVoltageRequest =
      new MotionMagicVelocityVoltage(0.0);

  private final VoltageOut wristVoltageRequest = new VoltageOut(0);
  private final PositionVoltage wristPositionRequest = new PositionVoltage(0.0);

  public IntakeIOTalonFX() {
    wheelMotor = new TalonFX(IntakeConstants.kIntakeWheelMotorId);
    wristMotor = new TalonFX(IntakeConstants.kIntakeWristMotorId);

    tryUntilOk(5, () -> wheelMotor.getConfigurator().apply(IntakeConstants.wheelMotorConfig, 0.25));
    tryUntilOk(5, () -> wristMotor.getConfigurator().apply(IntakeConstants.wristMotorConfig, 0.25));

    // Set up wheel motor status signals
    wheelAppliedVolts = wheelMotor.getMotorVoltage();
    wheelCurrent = wheelMotor.getStatorCurrent();
    wheelVelocity = wheelMotor.getVelocity();

    // Set up wrist motor status signals
    wristAppliedVolts = wristMotor.getMotorVoltage();
    wristCurrent = wristMotor.getStatorCurrent();
    wristPosition = wristMotor.getPosition();
    wristVelocity = wristMotor.getVelocity();

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        wheelAppliedVolts,
        wheelCurrent,
        wheelVelocity,
        wristAppliedVolts,
        wristCurrent,
        wristPosition,
        wristVelocity);
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    var wheelStatus = BaseStatusSignal.refreshAll(wheelAppliedVolts, wheelCurrent, wheelVelocity);
    inputs.wheelConnected = wheelStatus.equals(StatusCode.OK);
    inputs.wheelAppliedVolts = wheelAppliedVolts.getValueAsDouble();
    inputs.wheelCurrentAmps = wheelCurrent.getValueAsDouble();
    inputs.wheelVelocity = wheelVelocity.getValue().div(kWheelMotorToWheelGearRatio);

    var wristStatus =
        BaseStatusSignal.refreshAll(wristAppliedVolts, wristCurrent, wristPosition, wristVelocity);
    inputs.wristConnected = wristStatus.equals(StatusCode.OK);
    inputs.wristAppliedVolts = wristAppliedVolts.getValueAsDouble();
    inputs.wristCurrentAmps = wristCurrent.getValueAsDouble();
    inputs.wristPosition = wristPosition.getValue().div(kWristMotorToWristGearRatio);
    inputs.wristVelocity = wristVelocity.getValue().div(kWristMotorToWristGearRatio);
  }

  @Override
  public void setWheelOpenLoop(double volts) {
    wheelMotor.setControl(wheelVoltageRequest.withOutput(volts));
  }

  @Override
  public void setWheelVelocity(AngularVelocity velocity) {
    wheelMotor.setControl(
        wheelMotionMagicVelocityVoltageRequest
            .withVelocity(velocity.times(kWheelMotorToWheelGearRatio))
            .withAcceleration(Units.RotationsPerSecondPerSecond.of(100)));
  }

  @Override
  public void setWristOpenLoop(double volts) {
    wristMotor.setControl(wristVoltageRequest.withOutput(volts));
  }

  @Override
  public void setWristPosition(Angle angle) {
    wristMotor.setControl(
        wristPositionRequest.withPosition(angle.times(kWristMotorToWristGearRatio)));
  }

  @Override
  public void zeroWrist() {
    wristMotor.setPosition(0.0);
  }
}
