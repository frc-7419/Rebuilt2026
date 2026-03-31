package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import static frc.robot.subsystems.shooter.ShooterConstants.kMotorToShooterGearRatio;
import static frc.robot.subsystems.shooter.ShooterConstants.kShooterBangHandoffFraction;
import static frc.robot.subsystems.shooter.ShooterConstants.kShooterBangStatorAmps;
import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

public class ShooterIOTalonFX implements ShooterIO {
  private final TalonFX motor;
  private final TalonFX followerMotor;

  private final StatusSignal<Voltage> motorAppliedVolts;
  private final StatusSignal<Current> motorCurrent;
  private final StatusSignal<AngularVelocity> motorVelocity;

  private final VoltageOut voltageRequestLead = new VoltageOut(0);
  private final VoltageOut voltageRequestFollow = new VoltageOut(0);
  private final TorqueCurrentFOC torqueBangLead = new TorqueCurrentFOC(0);
  private final TorqueCurrentFOC torqueBangFollow = new TorqueCurrentFOC(0);
  private final VelocityVoltage velocityFocLead = new VelocityVoltage(0);
  private final VelocityVoltage velocityFocFollow = new VelocityVoltage(0);

  /** Target rotor velocity (RPS); NaN when open-loop only. */
  private double targetRotorRps = Double.NaN;

  public ShooterIOTalonFX() {
    motor = new TalonFX(ShooterConstants.kShooterMotorId);
    followerMotor = new TalonFX(ShooterConstants.kShooterFollowerMotorId);

    tryUntilOk(5, () -> motor.getConfigurator().apply(ShooterConstants.motorConfig, 0.25));
    tryUntilOk(5, () -> followerMotor.getConfigurator().apply(ShooterConstants.motorConfig, 0.25));

    motorAppliedVolts = motor.getMotorVoltage();
    motorCurrent = motor.getStatorCurrent();
    motorVelocity = motor.getVelocity();

    BaseStatusSignal.setUpdateFrequencyForAll(4.0, motorAppliedVolts, motorCurrent, motorVelocity);
  }

  private static boolean inBangPhase(double targetRps, double actualRps) {
    if (targetRps > 0) {
      return actualRps < kShooterBangHandoffFraction * targetRps;
    }
    if (targetRps < 0) {
      return actualRps > kShooterBangHandoffFraction * targetRps;
    }
    return false;
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    var status = BaseStatusSignal.refreshAll(motorAppliedVolts, motorCurrent, motorVelocity);
    inputs.connected = status.equals(StatusCode.OK);

    inputs.rotorVelocity = motorVelocity.getValue();
    inputs.shooterVelocity = motorVelocity.getValue().div(kMotorToShooterGearRatio);

    if (Double.isFinite(targetRotorRps)) {
      double targetRps = targetRotorRps;
      double actualRps = inputs.rotorVelocity.in(RotationsPerSecond);

      if (Math.abs(targetRps) < 1e-6) {
        motor.setControl(velocityFocLead.withVelocity(0).withEnableFOC(false));
        followerMotor.setControl(velocityFocFollow.withVelocity(0).withEnableFOC(false));
      } else if (inBangPhase(targetRps, actualRps)) {
        double bangAmps = Math.copySign(kShooterBangStatorAmps, targetRps);
        motor.setControl(torqueBangLead.withOutput(bangAmps));
        followerMotor.setControl(torqueBangFollow.withOutput(-bangAmps));
      } else {
        motor.setControl(velocityFocLead.withVelocity(targetRps).withEnableFOC(true));
        followerMotor.setControl(velocityFocFollow.withVelocity(-targetRps).withEnableFOC(true));
      }

      inputs.appliedVolts = motorAppliedVolts.getValueAsDouble();
      inputs.requestedVelocity = RotationsPerSecond.of(targetRotorRps);
    } else {
      inputs.appliedVolts = motorAppliedVolts.getValueAsDouble();
      inputs.requestedVelocity = RotationsPerSecond.of(0.0);
    }
    inputs.currentAmps = motorCurrent.getValueAsDouble();
  }

  @Override
  public void setOpenLoop(double volts) {
    targetRotorRps = Double.NaN;
    motor.setControl(voltageRequestLead.withOutput(volts).withEnableFOC(true));
    followerMotor.setControl(voltageRequestFollow.withOutput(-volts).withEnableFOC(true));
  }

  @Override
  public void setVelocity(AngularVelocity velocity) {
    targetRotorRps = velocity.in(RotationsPerSecond) * kMotorToShooterGearRatio;
  }

  @Override
  public void zeroRotor() {
    motor.setPosition(0.0);
    followerMotor.setPosition(0.0);
  }
}
