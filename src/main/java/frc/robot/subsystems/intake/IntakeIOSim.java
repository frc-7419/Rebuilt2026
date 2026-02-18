package frc.robot.subsystems.intake;

import static edu.wpi.first.math.MathUtil.*;
import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class IntakeIOSim implements IntakeIO {
  private static final double LOOP_PERIOD_SECS = 0.02;

  private static final double WHEEL_INERTIA = 0.001;
  private static final double WRIST_INERTIA = 0.001;
  private static final DCMotor WHEEL_MOTOR = DCMotor.getFalcon500(1);
  private static final DCMotor WRIST_MOTOR = DCMotor.getFalcon500(1);

  private final DCMotorSim wheelMotorSim;
  private final DCMotorSim wristMotorSim;

  private double wheelAppliedVolts = 0.0;
  private double wristAppliedVolts = 0.0;
  private double wristPosition = 0.0;

  public IntakeIOSim() {
    wheelMotorSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                WHEEL_MOTOR, WHEEL_INERTIA, IntakeConstants.kWheelMotorToWheelGearRatio),
            WHEEL_MOTOR);
    wristMotorSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                WRIST_MOTOR, WRIST_INERTIA, IntakeConstants.kWristMotorToWristGearRatio),
            WRIST_MOTOR);
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    wheelMotorSim.update(LOOP_PERIOD_SECS);
    wristMotorSim.update(LOOP_PERIOD_SECS);

    // Wheel motor inputs
    inputs.wheelConnected = true;
    inputs.wheelAppliedVolts = wheelAppliedVolts;
    inputs.wheelCurrentAmps = wheelMotorSim.getCurrentDrawAmps();
    inputs.wheelVelocity = RPM.of(wheelMotorSim.getAngularVelocityRPM());

    // Wrist motor inputs with position clamping
    inputs.wristConnected = true;
    inputs.wristAppliedVolts = wristAppliedVolts;
    inputs.wristCurrentAmps = wristMotorSim.getCurrentDrawAmps();

    // Clamp wrist position to mechanical limits
    wristPosition =
        clamp(
            wristPosition,
            IntakeConstants.kMinWristAngle.in(Degrees),
            IntakeConstants.kMaxWristAngle.in(Degrees));

    inputs.wristPosition = Degrees.of(wristPosition);
    inputs.wristVelocity =
        DegreesPerSecond.of(wristMotorSim.getAngularVelocityRadPerSec() * (180.0 / Math.PI));
  }

  @Override
  public void setWheelOpenLoop(double volts) {
    wheelAppliedVolts = clamp(volts, -12.0, 12.0);
    wheelMotorSim.setInputVoltage(wheelAppliedVolts);
  }

  @Override
  public void setWheelVelocity(AngularVelocity velocity) {
    // Simple proportional control for simulation
    double targetRPM = velocity.in(RPM);
    double currentRPM = wheelMotorSim.getAngularVelocityRPM();
    double error = targetRPM - currentRPM;
    double volts = error * IntakeConstants.kSimP;
    setWheelOpenLoop(volts);
  }

  @Override
  public void setWristOpenLoop(double volts) {
    wristAppliedVolts = clamp(volts, -12.0, 12.0);
    wristMotorSim.setInputVoltage(wristAppliedVolts);

    // Update wrist position
    wristPosition +=
        wristMotorSim.getAngularVelocityRadPerSec()
            * LOOP_PERIOD_SECS
            * (180.0 / Math.PI); // Convert rad/s to deg/s
  }

  @Override
  public void setWristPosition(Angle angle) {
    // Simple proportional control for simulation
    double targetDeg = angle.in(Degrees);
    double error = targetDeg - wristPosition;
    double volts = error * IntakeConstants.kSimP;
    setWristOpenLoop(volts);
  }

  @Override
  public void zeroWrist() {
    wristPosition = 0.0;
  }
}
