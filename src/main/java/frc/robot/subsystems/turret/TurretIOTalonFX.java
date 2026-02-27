package frc.robot.subsystems.turret;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static frc.robot.subsystems.turret.TurretConstants.kMotionProfile;
import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

/** TalonFX implementation of TurretIO. */
public class TurretIOTalonFX implements TurretIO {
  private final TalonFX motor;
  private final CANcoder rightEncoder;
  private final CANcoder leftEncoder;

  private final StatusSignal<Voltage> motorAppliedVolts;
  private final StatusSignal<Current> motorCurrent;
  private final StatusSignal<AngularVelocity> turretVelocity;
  private final StatusSignal<Angle> turretPosition;
  private final StatusSignal<Angle> rotorPosition;
  private final StatusSignal<Angle> rightEncoderPosition;
  private final StatusSignal<Angle> leftEncoderPosition;

  private Angle positionOffset = Degrees.of(0);

  // Control requests
  private final VoltageOut voltageRequest = new VoltageOut(0);
  private final PositionVoltage positionVoltageRequest = new PositionVoltage(0.0);

  private static final double kControlDt = 0.02;
  private boolean positionControl = false;
  private TrapezoidProfile.State goalState;

  public TurretIOTalonFX() {
    motor = new TalonFX(TurretConstants.kTurretMotorId);
    rightEncoder = new CANcoder(TurretConstants.kEncoderRightId);
    leftEncoder = new CANcoder(TurretConstants.kEncoderLeftId);

    tryUntilOk(5, () -> motor.getConfigurator().apply(TurretConstants.motorConfig, 0.25));
    tryUntilOk(5, () -> rightEncoder.getConfigurator().apply(TurretConstants.cancoderConfig, 0.25));
    tryUntilOk(5, () -> leftEncoder.getConfigurator().apply(TurretConstants.cancoderConfig, 0.25));

    motorAppliedVolts = motor.getMotorVoltage();
    motorCurrent = motor.getStatorCurrent();
    turretVelocity = motor.getVelocity();
    turretPosition = motor.getPosition();
    rotorPosition = motor.getRotorPosition();

    rightEncoderPosition = rightEncoder.getAbsolutePosition();
    leftEncoderPosition = leftEncoder.getAbsolutePosition();

    zeroRotor(Degrees.of(0));

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0, motorAppliedVolts, motorCurrent, turretVelocity, turretPosition, rotorPosition);

    BaseStatusSignal.setUpdateFrequencyForAll(50.0, rightEncoderPosition, leftEncoderPosition);
  }

  @Override
  public void updateInputs(TurretIOInputs inputs) {
    var status =
        BaseStatusSignal.refreshAll(
            motorAppliedVolts,
            motorCurrent,
            turretVelocity,
            turretPosition,
            rotorPosition,
            rightEncoderPosition,
            leftEncoderPosition);
    inputs.connected = status.equals(StatusCode.OK);
    inputs.appliedVolts = motorAppliedVolts.getValueAsDouble();
    inputs.currentAmps = motorCurrent.getValueAsDouble();
    inputs.velocity = turretVelocity.getValue();
    inputs.rotorPosition = rotorPosition.getValue();
    inputs.turretPosition = turretPosition.getValue().plus(positionOffset);
    inputs.rightEncoderPosition = rightEncoderPosition.getValue();
    inputs.leftEncoderPosition = leftEncoderPosition.getValue();

    if (positionControl && goalState != null) {
      TrapezoidProfile.State currentState =
          new TrapezoidProfile.State(
              turretPosition.getValue().in(Rotations),
              turretVelocity.getValue().in(RotationsPerSecond));
      TrapezoidProfile.State setpoint =
          kMotionProfile.calculate(kControlDt, currentState, goalState);
      positionVoltageRequest.Position = setpoint.position;
      positionVoltageRequest.Velocity = setpoint.velocity;

      inputs.requestedPosition = Rotations.of(setpoint.position).plus(positionOffset);
      inputs.requestedVelocity = RotationsPerSecond.of(setpoint.velocity);

      motor.setControl(positionVoltageRequest);
    }
  }

  @Override
  public void setOpenLoop(double volts) {
    positionControl = false;
    motor.setControl(voltageRequest.withOutput(volts));
  }

  @Override
  public void setPosition(Angle position) {
    setState(position, RPM.of(0));
  }

  @Override
  public void setState(Angle position, AngularVelocity velocity) {
    positionControl = true;
    goalState =
        new TrapezoidProfile.State(
            position.minus(positionOffset).in(Rotations), velocity.in(RotationsPerSecond));
  }

  @Override
  public void zeroRotor(Angle offset) {
    positionOffset = offset.minus(turretPosition.getValue());
  }
}
