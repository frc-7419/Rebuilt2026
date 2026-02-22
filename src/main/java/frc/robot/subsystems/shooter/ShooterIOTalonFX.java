package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.RadiansPerSecond;
import static frc.robot.subsystems.shooter.ShooterConstants.kMotorToShooterGearRatio;
import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

public class ShooterIOTalonFX implements ShooterIO {
  private final TalonFX motor;
  private final TalonFX followerMotor;

  private final StatusSignal<Voltage> motorAppliedVolts;
  private final StatusSignal<Current> motorCurrent;
  private final StatusSignal<AngularVelocity> motorVelocity;

  private final VoltageOut voltageRequest = new VoltageOut(0);
  private final VelocityVoltage velocityVoltageRequest = new VelocityVoltage(0.0);

  public ShooterIOTalonFX() {
    motor = new TalonFX(ShooterConstants.kShooterMotorId);
    followerMotor = new TalonFX(ShooterConstants.kShooterFollowerMotorId);

    tryUntilOk(5, () -> motor.getConfigurator().apply(ShooterConstants.motorConfig, 0.25));
    tryUntilOk(5, () -> followerMotor.getConfigurator().apply(ShooterConstants.motorConfig, 0.25));

    followerMotor.setControl(new Follower(motor.getDeviceID(), MotorAlignmentValue.Opposed));

    motorAppliedVolts = motor.getMotorVoltage();
    motorCurrent = motor.getStatorCurrent();
    motorVelocity = motor.getVelocity();

    BaseStatusSignal.setUpdateFrequencyForAll(50.0, motorAppliedVolts, motorCurrent, motorVelocity);
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    var status = BaseStatusSignal.refreshAll(motorAppliedVolts, motorCurrent, motorVelocity);
    inputs.connected = status.equals(StatusCode.OK);

    inputs.appliedVolts = motorAppliedVolts.getValueAsDouble();
    inputs.currentAmps = motorCurrent.getValueAsDouble();

    inputs.rotorVelocity = motorVelocity.getValue();
    inputs.shooterVelocity = motorVelocity.getValue().div(kMotorToShooterGearRatio);
    inputs.requestedVelocity = RadiansPerSecond.of(velocityVoltageRequest.Velocity);
  }

  @Override
  public void setOpenLoop(double volts) {
    motor.setControl(voltageRequest.withOutput(volts));
  }

  @Override
  public void setVelocity(AngularVelocity velocity) {
    motor.setControl(velocityVoltageRequest.withVelocity(velocity.div(kMotorToShooterGearRatio)));
  }

  @Override
  public void zeroRotor() {
    motor.setPosition(0.0);
    followerMotor.setPosition(0.0);
  }
}
