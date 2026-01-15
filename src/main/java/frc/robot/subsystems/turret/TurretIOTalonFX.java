private final TalonFX motor;
private final TalonFX launcher;
private final TalonFX shooter;

private final StatusSignal<Voltage> motorAppliedVolts, launcherAppliedVolts, shooterAppliedVolts;
private final StatusSignal<Current> motorCurrent, launcherCurrent, shooterCurrent;
private final StatusSignal<AngularVelocity> motorVelocity, launcherVelocity, shooterVelocity;
private final StatusSignal<edu.wpi.first.units.measure.Angle> motorPosition, launcherPosition, shooterPosition;

private final VoltageOut voltageRequest = new VoltageOut(0);
private final PositionVoltage positionVoltageRequest = new PositionVoltage(0.0);

public TurretIOTalonFX(int motorCanId, int launcherCanId, int shooterCanId) {
    motor = new TalonFX(motorCanId, TunerConstants.kCANBus);
    launcher = new TalonFX(launcherCanId, TunerConstants.kCANBus);
    shooter = new TalonFX(shooterCanId, TunerConstants.kCANBus);

    var config = new TalonFXConfiguration();
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    tryUntilOk(5, () -> motor.getConfigurator().apply(config, 0.25));
    tryUntilOk(5, () -> launcher.getConfigurator().apply(config, 0.25));
    tryUntilOk(5, () -> shooter.getConfigurator().apply(config, 0.25));

    motorAppliedVolts = motor.getMotorVoltage();
    motorCurrent = motor.getStatorCurrent();
    motorVelocity = motor.getVelocity();
    motorPosition = motor.getPosition();

    launcherAppliedVolts = launcher.getMotorVoltage();
    launcherCurrent = launcher.getStatorCurrent();
    launcherVelocity = launcher.getVelocity();
    launcherPosition = launcher.getPosition();

    shooterAppliedVolts = shooter.getMotorVoltage();
    shooterCurrent = shooter.getStatorCurrent();
    shooterVelocity = shooter.getVelocity();
    shooterPosition = shooter.getPosition();

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        motorAppliedVolts, motorCurrent, motorVelocity, motorPosition,
        launcherAppliedVolts, launcherCurrent, launcherVelocity, launcherPosition,
        shooterAppliedVolts, shooterCurrent, shooterVelocity, shooterPosition);
}

@Override
public void updateInputs(TurretIOInputs inputs) {
    var status = BaseStatusSignal.refreshAll(
        motorAppliedVolts, motorCurrent, motorVelocity, motorPosition,
        launcherAppliedVolts, launcherCurrent, launcherVelocity, launcherPosition,
        shooterAppliedVolts, shooterCurrent, shooterVelocity, shooterPosition);

    inputs.connected = status.equals(StatusCode.OK);

    inputs.appliedVolts = motorAppliedVolts.getValueAsDouble();
    inputs.currentAmps = motorCurrent.getValueAsDouble();
    inputs.velocityRadPerSec = Units.rotationsToRadians(motorVelocity.getValueAsDouble());
    inputs.position = Rotation2d.fromRotations(motorPosition.getValueAsDouble());
    inputs.absolutePosition = inputs.position;

    inputs.launcherVolts = launcherAppliedVolts.getValueAsDouble();
    inputs.launcherCurrent = launcherCurrent.getValueAsDouble();
    inputs.launcherVelocityRadPerSec = Units.rotationsToRadians(launcherVelocity.getValueAsDouble());
    inputs.launcherPosition = Rotation2d.fromRotations(launcherPosition.getValueAsDouble());

    inputs.shooterVolts = shooterAppliedVolts.getValueAsDouble();
    inputs.shooterCurrent = shooterCurrent.getValueAsDouble();
    inputs.shooterVelocityRadPerSec = Units.rotationsToRadians(shooterVelocity.getValueAsDouble());
    inputs.shooterPosition = Rotation2d.fromRotations(shooterPosition.getValueAsDouble());
}

@Override
public void setOpenLoop(double volts) {
    motor.setControl(voltageRequest.withOutput(volts));
}

@Override
public void setPosition(Rotation2d position) {
    motor.setControl(positionVoltageRequest.withPosition(position.getRotations()));
}

@Override
public void setLauncherOpenLoop(double volts) {
    launcher.setControl(voltageRequest.withOutput(volts));
}

@Override
public void setShooterOpenLoop(double volts) {
    shooter.setControl(voltageRequest.withOutput(volts));
}