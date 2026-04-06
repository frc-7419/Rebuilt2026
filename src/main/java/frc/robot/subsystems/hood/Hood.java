package frc.robot.subsystems.hood;

import static edu.wpi.first.wpilibj.Timer.getFPGATimestamp;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import frc.robot.RobotState;
import org.littletonrobotics.junction.Logger;

/** Hood subsystem */
public class Hood extends SubsystemBase {
  private final HoodIO io;
  private final HoodIOInputsAutoLogged inputs = new HoodIOInputsAutoLogged();
  private final Alert hoodDisconnectedAlert =
      new Alert("Disconnected hood motor.", AlertType.kError);

  public Hood(HoodIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Hood", inputs);

    double timestamp = getFPGATimestamp();
    RobotState.getInstance().addHoodUpdates(timestamp, inputs.position);

    hoodDisconnectedAlert.set(!inputs.connected && Constants.currentMode != Mode.SIM);
    RobotState.getInstance().setHoodDeviceConnected(inputs.connected);
  }

  /** Sets the hood in open loop (volts). Cancels any position hold. */
  public void setOpenLoop(double volts) {
    io.setOpenLoop(volts);
  }

  /** Set hood target angle */
  public void setAngle(Angle angle) {
    io.setPosition(angle);
  }

  /** Cancels position hold and stops the motor. */
  public void stop() {
    io.setOpenLoop(0.0);
  }

  /** Returns the current hood angle (0 = horizontal). */
  public Angle getAngle() {
    return inputs.position;
  }

  /** Returns the current hood angular velocity. */
  public AngularVelocity getVelocity() {
    return inputs.velocity;
  }

  /** Zero the rotor so current physical angle is treated as the initial offset */
  public void zeroRotor() {
    io.zeroRotor();
  }
}
