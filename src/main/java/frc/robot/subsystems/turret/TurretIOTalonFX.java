// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.turret;

import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.generated.TunerConstants;

/** TalonFX implementation of TurretIO. */
public class TurretIOTalonFX implements TurretIO {
  private final TalonFX motor;

  private final StatusSignal<Voltage> motorAppliedVolts;
  private final StatusSignal<Current> motorCurrent;
  private final StatusSignal<AngularVelocity> motorVelocity;
  private final StatusSignal<edu.wpi.first.units.measure.Angle> motorPosition;

  // Control requests
  private final VoltageOut voltageRequest = new VoltageOut(0);
  private final PositionVoltage positionVoltageRequest = new PositionVoltage(0.0);

  public TurretIOTalonFX(int canId) {
    motor = new TalonFX(canId, TunerConstants.kCANBus);

    var config = new TalonFXConfiguration();
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    tryUntilOk(5, () -> motor.getConfigurator().apply(config, 0.25));

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
    inputs.velocityRadPerSec = Units.rotationsToRadians(motorVelocity.getValueAsDouble());
    inputs.position = Rotation2d.fromRotations(motorPosition.getValueAsDouble());
    inputs.absolutePosition = inputs.position; // no separate absolute sensor
  }

  @Override
  public void setOpenLoop(double volts) {
    motor.setControl(voltageRequest.withOutput(volts));
  }

  @Override
  public void setPosition(Rotation2d position) {
    motor.setControl(positionVoltageRequest.withPosition(position.getRotations()));
  }
}
