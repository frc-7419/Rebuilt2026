package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static frc.robot.subsystems.shooter.ShooterConstants.computeVelocityVolts;
import static frc.robot.subsystems.shooter.ShooterConstants.kMotorToShooterGearRatio;
import static frc.robot.subsystems.shooter.ShooterConstants.kShooterKn;
import static frc.robot.subsystems.shooter.ShooterConstants.kShooterKv;
import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.Follower;
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

  /** Target mechanism velocity (rad/s); NaN when open-loop only. */
  private double targetVelocityRotPerSec = Double.NaN;

  public ShooterIOTalonFX() {
    motor = new TalonFX(ShooterConstants.kShooterMotorId);
    followerMotor = new TalonFX(ShooterConstants.kShooterFollowerMotorId);

    tryUntilOk(5, () -> motor.getConfigurator().apply(ShooterConstants.motorConfig, 0.25));
    tryUntilOk(5, () -> followerMotor.getConfigurator().apply(ShooterConstants.motorConfig, 0.25));

    followerMotor.setControl(new Follower(motor.getDeviceID(), MotorAlignmentValue.Opposed));

    motorAppliedVolts = motor.getMotorVoltage();
    motorCurrent = motor.getStatorCurrent();
    motorVelocity = motor.getVelocity();

    BaseStatusSignal.setUpdateFrequencyForAll(4.0, motorAppliedVolts, motorCurrent, motorVelocity);
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    var status = BaseStatusSignal.refreshAll(motorAppliedVolts, motorCurrent, motorVelocity);
    inputs.connected = status.equals(StatusCode.OK);

    inputs.rotorVelocity = motorVelocity.getValue();
    inputs.shooterVelocity = motorVelocity.getValue().div(kMotorToShooterGearRatio);

    if (Double.isFinite(targetVelocityRotPerSec)) {
      double targetRps = RotationsPerSecond.of(targetVelocityRotPerSec).in(RotationsPerSecond);
      double actualRps = inputs.rotorVelocity.in(RotationsPerSecond);
      double volts = computeVelocityVolts(targetRps, actualRps, kShooterKv, kShooterKn);
      motor.setControl(voltageRequest.withOutput(volts));
      inputs.appliedVolts = volts;
      inputs.requestedVelocity = RotationsPerSecond.of(targetVelocityRotPerSec);
    } else {
      inputs.appliedVolts = motorAppliedVolts.getValueAsDouble();
      inputs.requestedVelocity = RPM.of(0.0);
    }
    inputs.currentAmps = motorCurrent.getValueAsDouble();
  }

  @Override
  public void setOpenLoop(double volts) {
    targetVelocityRotPerSec = Double.NaN;
    motor.setControl(voltageRequest.withOutput(volts));
  }

  @Override
  public void setVelocity(AngularVelocity velocity) {
    targetVelocityRotPerSec = velocity.in(RotationsPerSecond) * kMotorToShooterGearRatio;
  }

  @Override
  public void zeroRotor() {
    motor.setPosition(0.0);
    followerMotor.setPosition(0.0);
  }
}
