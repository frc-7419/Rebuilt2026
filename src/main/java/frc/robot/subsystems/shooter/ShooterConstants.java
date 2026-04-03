package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RPM;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;

public final class ShooterConstants {
  private ShooterConstants() {}

  public static final int kShooterMotorId = 40;
  public static final int kShooterFollowerMotorId = 38;

  public static final double kMaxVoltage = 12.0;

  public static final AngularVelocity kMinVelocity = RPM.of(0.0);
  public static final AngularVelocity kMaxVelocity = RPM.of(6000.0);

  public static final double kDeadband = 0.05;

  // Main/Secondary flywheels: 20:36 reduction
  public static final double kMotorToShooterGearRatio = (20.0 / 36.0);

  public static final TalonFXConfiguration motorConfig = new TalonFXConfiguration();
  public static final Slot0Configs motorSlot0Configs = motorConfig.Slot0;

  /** kV in V/RPS */
  public static final double kShooterKv = 0.13;

  public static final double kShooterKp = 0.4;
  public static final double kShooterKs = 0.467;

  public static final double kShooterBangHandoffFraction = 0.95;
  public static final double kShooterBangStatorAmps = 80;

  static {
    motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    motorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    motorSlot0Configs.kP = kShooterKp;
    motorSlot0Configs.kI = 0;
    motorSlot0Configs.kD = 0;
    motorSlot0Configs.kV = kShooterKv;
    motorSlot0Configs.kS = kShooterKs;
  }

  private static final DCMotor kSimMotor = DCMotor.getKrakenX60Foc(2);
  private static final double kTwoPi = 2.0 * Math.PI;

  /** Ideal kV for sim */
  public static final double kSimKv =
      12.0 / (kSimMotor.freeSpeedRadPerSec * kMotorToShooterGearRatio / kTwoPi);

  public static final double kSimKp = 0.4;
  public static final double kSimKs = 0;

  public static double computeVelocityVolts(
      double targetRps, double actualRps, double kv, double kP, double kS) {
    double negErrorRps = Math.max(0.0, targetRps - actualRps);
    double volts = kv * targetRps + kP * negErrorRps + kS;
    return MathUtil.clamp(volts, -kMaxVoltage, kMaxVoltage);
  }

  public static double computeVelocityVolts(
      double targetRps, double actualRps, double kv, double kP) {
    return computeVelocityVolts(targetRps, actualRps, kv, kP, 0.0);
  }

  public static final Angle kHoodZeroed = Degrees.of(0.0);

  public static final Translation3d kRobotToShooterRelease = new Translation3d(0.0, 0.0, 0.0);

  public static final Distance kShooterWheelRadius = Meters.of(0.050);

  /** Estimated velocity, tune to robot */
  public static final double kFuelLaunchVelMetersPerSecPerRotPerSec = 0.18;

  /** Default RPM for pass shots */
  public static final double kAutoAimRPM = 3000.0;

  /** Default RPM for idle state. */
  public static final double kIdleRPM = 2000.0;

  /** Min speed before firing. */
  public static final double kAutoAimRPMMin = 1000.0;

  /** Hardware bound */
  public static final double kAutoAimRPMMax = 6000.0;

  /** RPM tolerance for "at speed" */
  public static final double kRpmToleranceForReady = 300.0;
}
