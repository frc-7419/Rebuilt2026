package frc.robot.subsystems.hopper;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

/** Simulation implementation of HopperIO. */
public class HopperIOSim implements HopperIO {
  private static final double LOOP_PERIOD_SECS = 0.02;

  private static final double HOPPER_INERTIA = 0.001;
  private static final DCMotor MOTOR_MODEL = DCMotor.getFalcon500(1);

  private final DCMotorSim motorSim;

  private double appliedVolts = 0.0;

  public HopperIOSim() {
    motorSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                MOTOR_MODEL, HOPPER_INERTIA, HopperConstants.kMotorToHopperGearRatio),
            MOTOR_MODEL);
  }

  @Override
  public void updateInputs(HopperIOInputs inputs) {
    motorSim.update(LOOP_PERIOD_SECS);

    inputs.connected = true;
    inputs.appliedVolts = appliedVolts;
    inputs.currentAmps = motorSim.getCurrentDrawAmps();
    inputs.velocity = edu.wpi.first.units.Units.RPM.of(motorSim.getAngularVelocityRPM());
  }

  @Override
  public void setOpenLoop(double volts) {
    appliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
    motorSim.setInputVoltage(appliedVolts);
  }

  @Override
  public void setVelocity(AngularVelocity velocity) {
    // Simple proportional control for simulation
    double targetRPM = velocity.in(edu.wpi.first.units.Units.RPM);
    double currentRPM = motorSim.getAngularVelocityRPM();
    double error = targetRPM - currentRPM;
    double volts = error * HopperConstants.kSimP;
    setOpenLoop(volts);
  }
}
