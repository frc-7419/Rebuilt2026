package frc.robot.subsystems.hood;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static frc.robot.subsystems.hood.HoodConstants.kDegreesPerMotorRotation;
import static frc.robot.subsystems.hood.HoodConstants.kInitialAngleOffsetDeg;
import static frc.robot.subsystems.hood.HoodConstants.kMotorToHoodGearRatio;
import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

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

    double rotorRot = motorPosition.getValue().in(Rotations);
    double rotorRotPerSec = motorVelocity.getValue().in(RotationsPerSecond);

    double positionDeg = kInitialAngleOffsetDeg - rotorRot * kDegreesPerMotorRotation;
    inputs.position = Degrees.of(positionDeg);

    double hoodRadPerSec = -rotorRotPerSec * (2.0 * Math.PI / kMotorToHoodGearRatio);
    inputs.velocity = RadiansPerSecond.of(hoodRadPerSec);
  }

  @Override
  public void setOpenLoop(double volts) {
    motor.setControl(voltageRequest.withOutput(volts));
  }

  @Override
  public void setPosition(Angle position) {
    double targetDeg = position.in(Degrees);
    double targetRotorRot = (kInitialAngleOffsetDeg - targetDeg) / kDegreesPerMotorRotation;
    motor.setControl(motionMagicRequest.withPosition(targetRotorRot));
  }

  @Override
  public void zeroRotor() {
    motor.setPosition(0.0);
  }
}
