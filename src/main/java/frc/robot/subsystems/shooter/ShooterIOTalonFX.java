package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import static frc.robot.subsystems.shooter.ShooterConstants.kMotorToShooterGearRatio;
import static frc.robot.subsystems.shooter.ShooterConstants.kShooterBangHandoffFraction;
import static frc.robot.subsystems.shooter.ShooterConstants.kShooterBangStatorAmps;
import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import java.util.function.Consumer;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class ShooterIOTalonFX implements ShooterIO {
  private final TalonFX motor;
  private final TalonFX followerMotor;

  private final StatusSignal<Voltage> motorAppliedVolts;
  private final StatusSignal<Current> motorCurrent;
  private final StatusSignal<AngularVelocity> motorVelocity;

  private final VoltageOut voltageRequestLead = new VoltageOut(0);
  private final VoltageOut voltageRequestFollow = new VoltageOut(0);
  private final ClosedLoop closedLoop;

  public ShooterIOTalonFX() {
    motor = new TalonFX(ShooterConstants.kShooterMotorId);
    followerMotor = new TalonFX(ShooterConstants.kShooterFollowerMotorId);

    tryUntilOk(5, () -> motor.getConfigurator().apply(ShooterConstants.motorConfig, 0.25));
    tryUntilOk(5, () -> followerMotor.getConfigurator().apply(ShooterConstants.motorConfig, 0.25));

    motorAppliedVolts = motor.getMotorVoltage();
    motorCurrent = motor.getStatorCurrent();
    motorVelocity = motor.getVelocity();
    closedLoop =
        new ClosedLoop(
            request -> motor.setControl(request),
            request -> followerMotor.setControl(request),
            motorVelocity::getValueAsDouble);

    BaseStatusSignal.setUpdateFrequencyForAll(50.0, motorAppliedVolts, motorCurrent, motorVelocity);
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    var status = BaseStatusSignal.refreshAll(motorAppliedVolts, motorCurrent, motorVelocity);
    inputs.connected = status.equals(StatusCode.OK);
    inputs.appliedVolts = motorAppliedVolts.getValueAsDouble();

    inputs.rotorVelocity = motorVelocity.getValue();
    inputs.shooterVelocity = motorVelocity.getValue().div(kMotorToShooterGearRatio);

    closedLoop.apply();
    inputs.requestedVelocity =
        RotationsPerSecond.of(
            Double.isFinite(closedLoop.targetRotorRps) ? closedLoop.targetRotorRps : 0.0);
    inputs.currentAmps = motorCurrent.getValueAsDouble();
  }

  @Override
  public void setOpenLoop(double volts) {
    closedLoop.targetRotorRps = Double.NaN;
    motor.setControl(voltageRequestLead.withOutput(volts).withEnableFOC(true));
    followerMotor.setControl(voltageRequestFollow.withOutput(-volts).withEnableFOC(true));
  }

  @Override
  public void setVelocity(AngularVelocity velocity) {
    closedLoop.setWheelRps(velocity.in(RotationsPerSecond));
  }

  @Override
  public void zeroRotor() {
    motor.setPosition(0.0);
    followerMotor.setPosition(0.0);
  }

  /** Hardware-independent transport seam for checking timing, handoff, and follower direction. */
  static final class ClosedLoop {
    private final Consumer<ControlRequest> lead;
    private final Consumer<ControlRequest> follow;
    private final DoubleSupplier actualRotorRps;
    private final TorqueCurrentFOC torqueBangLead = new TorqueCurrentFOC(0);
    private final TorqueCurrentFOC torqueBangFollow = new TorqueCurrentFOC(0);
    private final VelocityVoltage velocityFocLead = new VelocityVoltage(0);
    private final VelocityVoltage velocityFocFollow = new VelocityVoltage(0);
    private double targetRotorRps = Double.NaN;

    ClosedLoop(
        Consumer<ControlRequest> lead,
        Consumer<ControlRequest> follow,
        DoubleSupplier actualRotorRps) {
      this.lead = lead;
      this.follow = follow;
      this.actualRotorRps = actualRotorRps;
    }

    void setWheelRps(double wheelRps) {
      targetRotorRps = wheelRps * kMotorToShooterGearRatio;
      apply();
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

    void apply() {
      if (!Double.isFinite(targetRotorRps)) return;
      double targetRps = targetRotorRps;
      double actualRps = actualRotorRps.getAsDouble();
      if (Math.abs(targetRps) < 1e-6) {
        lead.accept(velocityFocLead.withVelocity(0).withEnableFOC(false));
        follow.accept(velocityFocFollow.withVelocity(0).withEnableFOC(false));
      } else if (inBangPhase(targetRps, actualRps)) {
        double bangAmps = Math.copySign(kShooterBangStatorAmps, targetRps);
        lead.accept(torqueBangLead.withOutput(bangAmps));
        follow.accept(torqueBangFollow.withOutput(-bangAmps));
        Logger.recordOutput("Shooter/BangPhase", true);
      } else {
        lead.accept(velocityFocLead.withVelocity(targetRps).withEnableFOC(true));
        follow.accept(velocityFocFollow.withVelocity(-targetRps).withEnableFOC(true));
        Logger.recordOutput("Shooter/BangPhase", false);
      }
    }
  }
}
