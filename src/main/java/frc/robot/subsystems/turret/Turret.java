package frc.robot.subsystems.turret;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotState;
import org.littletonrobotics.junction.Logger;
import yams.units.EasyCRT;
import yams.units.EasyCRTConfig;

/** Turret subsystem implementing position control with IO abstraction. */
public class Turret extends SubsystemBase {
  private final TurretIO io;
  private final TurretIOInputsAutoLogged inputs = new TurretIOInputsAutoLogged();
  private final EasyCRTConfig config;
  private boolean seeded;

  private static final double kSeedTimeoutSec = 30.0;
  private static final double kSeedRetryPeriodSec = 10.0;

  private final double seedStartTimeSec = Timer.getFPGATimestamp();
  private double lastSeedAttemptSec = -Double.MAX_VALUE;
  private boolean seedTimedOut = false;

  public Turret(TurretIO io) {
    this.io = io;
    config =
        new EasyCRTConfig(() -> inputs.rightEncoderPosition, () -> inputs.leftEncoderPosition)
            .withEncoderRatios(
                TurretConstants.kMotorToTurretGearRatio
                    / TurretConstants.kMotorToRightEncoderGearRatio,
                TurretConstants.kMotorToTurretGearRatio
                    / TurretConstants.kMotorToLeftEncoderGearRatio)
            .withMechanismRange(
                TurretConstants.kAbsoluteMinAngle, TurretConstants.kAbsoluteMaxAngle)
            .withAbsoluteEncoderInversions(false, true)
            .withAbsoluteEncoderOffsets(
                TurretConstants.rightEncoderZeroOffset, TurretConstants.leftEncoderZeroOffset);

    Logger.recordOutput("Turret/SatisfiesRange", config.coverageSatisfiesRange());
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Turret", inputs);

    double timestamp = Timer.getFPGATimestamp();
    RobotState.getInstance().addTurretUpdates(timestamp, inputs.turretPosition, inputs.velocity);

    if (!seeded && !seedTimedOut) {
      double elapsed = timestamp - seedStartTimeSec;

      if (elapsed > kSeedTimeoutSec) {
        seedTimedOut = true;
        Logger.recordOutput("Turret/CRTSolverStatus", "Seed Timeout");
      } else if (timestamp - lastSeedAttemptSec >= kSeedRetryPeriodSec) {
        lastSeedAttemptSec = timestamp;
        seeded = seed();
      }

      Logger.recordOutput("Turret/SeedElapsedSec", elapsed);
      Logger.recordOutput("Turret/Seeded", seeded);
    }
  }

  /** Sets the turret in open loop (volts). Cancels any position hold. */
  public void setOpenLoop(double volts) {
    io.setOpenLoop(volts);
  }

  public void setAngle(Angle angle) {
    Logger.recordOutput("Turret/TurretRequestedDeg", angle.in(Degrees));
    double angleRad = angle.in(Radians);

    angleRad =
        MathUtil.clamp(
            angleRad,
            TurretConstants.kAbsoluteMinAngle.in(Radians),
            TurretConstants.kAbsoluteMaxAngle.in(Radians));

    Logger.recordOutput("Turret/TurretRequestedRad", angleRad);
    io.setPosition(Radians.of(angleRad));
  }

  /** Cancels any position hold and stops the motor. */
  public void stop() {
    io.setOpenLoop(0.0);
  }

  /** Returns the most recent turret angle. */
  public Angle getAngle() {
    return inputs.turretPosition;
  }

  public boolean seed() {
    if (Constants.currentMode != Constants.Mode.REAL) return true;

    if (inputs.velocity.in(RadiansPerSecond) > 0.1) {
      Logger.recordOutput("Turret/CRTSolverStatus", "Turret in Motion");
      return false;
    }

    EasyCRT easyCrtSolver = new EasyCRT(config);

    easyCrtSolver
        .getAngleOptional()
        .ifPresent(
            angle -> {
              io.zeroRotor(angle);
            });

    String status = easyCrtSolver.getLastStatus();
    Logger.recordOutput("Turret/CRTSolverStatus", status);
    return easyCrtSolver.getAngleOptional().isPresent();
  }
}
