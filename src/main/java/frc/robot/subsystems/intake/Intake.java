package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import frc.robot.RobotState;
import org.littletonrobotics.junction.Logger;

/** Intake subsystem with wheel and wrist motors. */
public class Intake extends SubsystemBase {
  private final IntakeIO io;
  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();
  private final Alert wheelDisconnectedAlert =
      new Alert("Disconnected intake wheel motor.", AlertType.kError);
  private final Alert wristDisconnectedAlert =
      new Alert("Disconnected intake wrist motor.", AlertType.kError);

  public Intake(IntakeIO io) {
    this.io = io;
    io.zeroWrist();
  }

  /** Wrist angle (degrees) above this is considered "down" (deployed) for intake. */
  private static final double kIntakeDownThresholdDeg = 90.0;

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Intake", inputs);

    double timestamp = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();
    RobotState state = RobotState.getInstance();
    state.addIntakeUpdates(timestamp, inputs.wristPosition, inputs.wristVelocity);
    state.addIntakeWheelUpdates(timestamp, inputs.wheelVelocity);
    state.setIntakeWheelAppliedVolts(inputs.wheelAppliedVolts);
    state.setIntakeDeviceConnections(inputs.wheelConnected, inputs.wristConnected);
    state.setIntakeDown(inputs.wristPosition.in(Degrees) >= kIntakeDownThresholdDeg);

    if (Constants.currentMode != Mode.SIM) {
      wheelDisconnectedAlert.set(!inputs.wheelConnected);
      wristDisconnectedAlert.set(!inputs.wristConnected);
    }
  }

  /** Set intake wheel motor in open loop (volts). */
  public void setWheelOpenLoop(double volts) {
    io.setWheelOpenLoop(volts);
  }

  /** Set intake wheel motor velocity control. */
  public void setWheelVelocity(AngularVelocity velocity) {
    io.setWheelVelocity(velocity);
  }

  /** Stop the intake wheel motor. */
  public void stopWheel() {
    io.setWheelOpenLoop(0.0);
  }

  /** Get intake wheel velocity. */
  public AngularVelocity getWheelVelocity() {
    return inputs.wheelVelocity;
  }

  /** Set intake wrist in open loop (volts). */
  public void setWristOpenLoop(double volts) {
    io.setWristOpenLoop(volts);
  }

  /** Set intake wrist position control. */
  public void setWristAngle(Angle angle) {
    double deg = angle.in(Degrees);
    double clamped =
        MathUtil.clamp(
            deg,
            IntakeConstants.kMinWristAngle.in(Degrees),
            IntakeConstants.kMaxWristAngle.in(Degrees));
    Logger.recordOutput("Intake/WristRequestedDeg", clamped);
    io.setWristPosition(Degrees.of(clamped));
  }

  /** Stop the intake wrist motor. */
  public void stopWrist() {
    io.setWristOpenLoop(0.0);
  }

  /** Get intake wrist angle. */
  public Angle getWristAngle() {
    return inputs.wristPosition;
  }

  /** Get intake wrist velocity. */
  public AngularVelocity getWristVelocity() {
    return inputs.wristVelocity;
  }

  /** Zero the wrist encoder. */
  public void zeroWrist() {
    io.zeroWrist();
  }
}
