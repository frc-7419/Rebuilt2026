package frc.robot.subsystems.shooter;

import static edu.wpi.first.math.MathUtil.clamp;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;
import static edu.wpi.first.wpilibj.Timer.getFPGATimestamp;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import frc.robot.RobotState;
import org.littletonrobotics.junction.Logger;

/** Shooter subsystem implementing velocity control with IO abstraction. */
public class Shooter extends SubsystemBase {
  private final ShooterIO io;
  private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();
  private final Alert shooterDisconnectedAlert =
      new Alert("Disconnected shooter motor.", AlertType.kError);

  /** Last voltage commanded by {@link #runCharacterization(double)} (SysId). */
  private double sysIdAppliedVolts = 0.0;

  private final SysIdRoutine sysId;

  public Shooter(ShooterIO io) {
    this.io = io;

    sysId =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                null,
                null,
                (state) -> Logger.recordOutput("Shooter/SysIdState", state.toString())),
            new SysIdRoutine.Mechanism(
                (voltage) -> runCharacterization(voltage.in(Volts)),
                log ->
                    log.motor("shooter-main")
                        .voltage(Volts.of(sysIdAppliedVolts))
                        .angularVelocity(inputs.rotorVelocity),
                this));
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Shooter", inputs);
    Logger.recordOutput("Shooter/RequestedRPM", inputs.requestedVelocity.in(RPM));

    double timestamp = getFPGATimestamp();
    RobotState state = RobotState.getInstance();
    state.addShooterUpdates(timestamp, inputs.shooterVelocity, inputs.rotorVelocity);
    double currentRpm = inputs.shooterVelocity.in(RPM);
    double requestedRpm = inputs.requestedVelocity.in(RPM);
    boolean atSpeed =
        requestedRpm > 100
            && Math.abs(currentRpm - requestedRpm) <= ShooterConstants.kRpmToleranceForReady;
    state.setShooterRpmInRange(atSpeed);

    shooterDisconnectedAlert.set(!inputs.connected && Constants.currentMode != Mode.SIM);
    state.setShooterDeviceConnected(inputs.connected);
  }

  /** Sets the shooter in open loop (volts). */
  public void setOpenLoop(double volts) {
    io.setOpenLoop(volts);
  }

  /** Set shooter target velocity. */
  public void setVelocity(AngularVelocity velocity) {
    double targetRadPerSec = velocity.in(RadiansPerSecond);
    double clamped =
        clamp(
            targetRadPerSec,
            ShooterConstants.kMinVelocity.in(RadiansPerSecond),
            ShooterConstants.kMaxVelocity.in(RadiansPerSecond));
    io.setVelocity(RadiansPerSecond.of(clamped));
  }

  /** Returns the last requested RPM from the IO (set via {@link #setVelocity}). */
  public double getRequestedRPM() {
    return inputs.requestedVelocity.in(RPM);
  }

  /** Cancels any velocity hold and stops the motor. */
  public void stop() {
    io.setOpenLoop(0.0);
  }

  /**
   * Runs the shooter in open loop at the given voltage for SysId characterization. Uses motor-side
   * velocity in the SysId log ({@code inputs.rotorVelocity}) so gains match Phoenix {@code kV} /
   * {@code kS} in rotor RPS.
   */
  public void runCharacterization(double output) {
    sysIdAppliedVolts = output;
    io.setOpenLoop(output);
  }

  /** Returns a command to run a quasistatic test in the specified direction. */
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return run(() -> runCharacterization(0.0))
        .withTimeout(1.0)
        .andThen(sysId.quasistatic(direction));
  }

  /** Returns a command to run a dynamic test in the specified direction. */
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return run(() -> runCharacterization(0.0)).withTimeout(1.0).andThen(sysId.dynamic(direction));
  }

  /** Returns the most recent shooter wheel velocity. */
  public AngularVelocity getVelocity() {
    return inputs.shooterVelocity;
  }

  public double getRPM() {
    return inputs.shooterVelocity.in(RPM);
  }

  public double getRotorRPM() {
    return inputs.rotorVelocity.in(RPM);
  }
}
