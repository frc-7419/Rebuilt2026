package frc.robot.subsystems.turret;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
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
  private final Alert turretDisconnectedAlert =
      new Alert("Disconnected turret motor.", AlertType.kError);

  /** Radians; must match logic in {@link #runCharacterization(double)}. */
  private static final double kCharacterizationLimitMarginRad = Units.degreesToRadians(3.0);

  public Turret(TurretIO io) {
    this.io = io;
    config =
        new EasyCRTConfig(() -> inputs.rightEncoderPosition, () -> inputs.leftEncoderPosition)
            .withCommonDriveGear((130.0 / 10.0) * (60.0 / 30.0), 30, 17, 16)
            .withMechanismRange(Degrees.of(-60), Degrees.of(60))
            .withAbsoluteEncoderInversions(false, true)
            .withMatchTolerance(Rotations.of(0.06))
            .withAbsoluteEncoderOffsets(
                TurretConstants.rightEncoderZeroOffset, TurretConstants.leftEncoderZeroOffset);

    Logger.recordOutput("Turret/SatisfiesRange", config.coverageSatisfiesRange());
    config
        .getUniqueCoverage()
        .ifPresent(
            angle -> {
              Logger.recordOutput("Turret/UniqueCoverage", angle.in(Degrees));
            });
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Turret", inputs);

    double timestamp = Timer.getFPGATimestamp();
    RobotState.getInstance()
        .addTurretUpdates(
            timestamp, inputs.turretPosition, inputs.velocity, inputs.requestedPosition);

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

    turretDisconnectedAlert.set(!inputs.connected && Constants.currentMode != Mode.SIM);
    RobotState.getInstance().setTurretDeviceConnected(inputs.connected);
  }

  /** Sets the turret in open loop (volts). Cancels any position hold. */
  public void setOpenLoop(double volts) {
    io.setOpenLoop(volts);
  }

  /**
   * Open-loop voltage for simple FF characterization. Voltage is clamped near absolute limits so we
   * do not command into the stops (same sign convention as {@link TurretIOSim}).
   */
  public void runCharacterization(double voltage) {
    double theta = getAngle().in(Radians);
    double min = TurretConstants.kAbsoluteMinAngle.in(Radians);
    double max = TurretConstants.kAbsoluteMaxAngle.in(Radians);
    double v = MathUtil.clamp(voltage, -TurretConstants.kMaxVoltage, TurretConstants.kMaxVoltage);
    if (theta <= min + kCharacterizationLimitMarginRad) {
      v = Math.max(0.0, v);
    }
    if (theta >= max - kCharacterizationLimitMarginRad) {
      v = Math.min(0.0, v);
    }
    setOpenLoop(v);
  }

  /**
   * Drives toward the minimum soft limit with fixed negative voltage (slow approach). Stops output
   * when {@link #isNearMinLimitForCharacterization()} is true. Does not command past the min side
   * of the margin band.
   */
  public void runSeekToMinForCharacterization(double voltsMagnitude) {
    double theta = getAngle().in(Radians);
    double min = TurretConstants.kAbsoluteMinAngle.in(Radians);
    if (theta <= min + kCharacterizationLimitMarginRad) {
      setOpenLoop(0.0);
      return;
    }
    double v = -MathUtil.clamp(Math.abs(voltsMagnitude), 0.0, TurretConstants.kMaxVoltage);
    setOpenLoop(v);
  }

  /** True when at or inside the min-side margin used for characterization. */
  public boolean isNearMinLimitForCharacterization() {
    double theta = getAngle().in(Radians);
    double min = TurretConstants.kAbsoluteMinAngle.in(Radians);
    return theta <= min + kCharacterizationLimitMarginRad;
  }

  /** Mechanism angular velocity in rotations per second (matches drive FF / Phoenix kV units). */
  public double getFFCharacterizationVelocity() {
    return inputs.velocity.in(RotationsPerSecond);
  }

  public void setAngle(Angle angle) {
    setAngleWithVelocity(angle, RadiansPerSecond.of(0.0));
  }

  /** Sets turret angle with velocity feedforward */
  public void setAngleWithVelocity(Angle angle, AngularVelocity velocityRadPerSec) {
    double angleRad = angle.in(Radians);
    angleRad =
        MathUtil.clamp(
            angleRad,
            TurretConstants.kAbsoluteMinAngle.in(Radians),
            TurretConstants.kAbsoluteMaxAngle.in(Radians));
    Logger.recordOutput("Turret/TurretRequestedDeg", angleRad * 180.0 / Math.PI);
    Logger.recordOutput(
        "Turret/TurretRequestedVelRadPerSec", velocityRadPerSec.baseUnitMagnitude());
    io.setState(Radians.of(angleRad), velocityRadPerSec);
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
              Logger.recordOutput("Turret/CRTSolverAngleDeg", (angle.in(Degrees)));
              io.zeroRotor(angle);
            });

    String status = easyCrtSolver.getLastStatus();
    Logger.recordOutput("Turret/CRTSolverStatus", status);

    return easyCrtSolver.getAngleOptional().isPresent();
  }

  public void zeroRotor(Angle offset) {
    io.zeroRotor(offset);
  }
}
