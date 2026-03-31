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

  static {
    motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    motorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    motorSlot0Configs.kP = 0.6;
    motorSlot0Configs.kI = 0;
    motorSlot0Configs.kD = 0;
    motorSlot0Configs.kV = 0.1425;
    motorSlot0Configs.kS = 0.2;
  }

  /** kV in V/RPS */
  public static final double kShooterKv = 0.13;

  public static final double kShooterKp = 0.4;
  public static final double kShooterKs = 0.47;

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
  public static final double kFuelLaunchVelMetersPerSecPerRotPerSec = 0.22;

  /** Default RPM for pass shots */
  public static final double kAutoAimRPM = 3000.0;

  /** Default RPM for idle state. */
  public static final double kIdleRPM = 2000.0;

  /** Min speed before firing. */
  public static final double kAutoAimRPMMin = 1000.0;

  /** Hardware bound */
  public static final double kAutoAimRPMMax = 3500.0;

  /** RPM tolerance for "at speed" */
  public static final double kRpmToleranceForReady = 200.0;

  /**
   * Nonlinear kV lookup table for shooter velocity control.
   * Maps RPM values to kV (voltage feedforward) constants.
   * Accounts for nonlinearity from friction at low speeds and back-EMF at high speeds.
   *
   * Each entry represents the optimal kV for that RPM range.
   * Entries are indexed by RPM in ~120 RPM increments across 0-6000 RPM range.
   */
  public static final double[] kV_LOOKUP_RPM = {
      0, 120, 240, 360, 480, 600, 720, 840, 960, 1080,
      1200, 1320, 1440, 1560, 1680, 1800, 1920, 2040, 2160, 2280,
      2400, 2520, 2640, 2760, 2880, 3000, 3120, 3240, 3360, 3480,
      3600, 3720, 3840, 3960, 4080, 4200, 4320, 4440, 4560, 4680,
      4800, 4920, 5040, 5160, 5280, 5400, 5520, 5640, 5760, 5880, 6000
  };

  /**
   * kV (V/RPS) values corresponding to kV_LOOKUP_RPM entries.
   * Values range from ~0.165 at low speeds (high friction) to ~0.120 at high speeds (back-EMF).
   */
  public static final double[] kV_LOOKUP_VALUES = {
      0.1650, 0.1620, 0.1590, 0.1560, 0.1535, 0.1510, 0.1485, 0.1460, 0.1440, 0.1420,
      0.1400, 0.1385, 0.1370, 0.1360, 0.1350, 0.1340, 0.1332, 0.1325, 0.1318, 0.1312,
      0.1306, 0.1302, 0.1298, 0.1294, 0.1290, 0.1287, 0.1284, 0.1281, 0.1278, 0.1275,
      0.1272, 0.1268, 0.1264, 0.1260, 0.1256, 0.1250, 0.1245, 0.1240, 0.1234, 0.1228,
      0.1222, 0.1216, 0.1210, 0.1204, 0.1198, 0.1192, 0.1186, 0.1180, 0.1174, 0.1168, 0.1162
  };

  /**
   * Interpolates kV value for a given RPM using the lookup table.
   * Uses linear interpolation between table entries.
   *
   * @param rpm Target shooter RPM (in 1/min, not RPS)
   * @return Interpolated kV value in V/RPS
   */
  public static double getInterpolatedKV(double rpm) {
    rpm = MathUtil.clamp(rpm, 0.0, 6000.0);

    // Find the two surrounding entries in the lookup table
    int lowIndex = 0;
    for (int i = 0; i < kV_LOOKUP_RPM.length - 1; i++) {
      if (kV_LOOKUP_RPM[i] <= rpm && rpm < kV_LOOKUP_RPM[i + 1]) {
        lowIndex = i;
        break;
      }
      if (kV_LOOKUP_RPM[i] > rpm) {
        lowIndex = Math.max(0, i - 1);
        break;
      }
    }

    int highIndex = Math.min(lowIndex + 1, kV_LOOKUP_RPM.length - 1);

    // Linear interpolation
    double rpmLow = kV_LOOKUP_RPM[lowIndex];
    double rpmHigh = kV_LOOKUP_RPM[highIndex];
    double kvLow = kV_LOOKUP_VALUES[lowIndex];
    double kvHigh = kV_LOOKUP_VALUES[highIndex];

    if (rpmLow == rpmHigh) {
      return kvLow;
    }

    double t = (rpm - rpmLow) / (rpmHigh - rpmLow);
    return kvLow + t * (kvHigh - kvLow);
  }

  /**
   * Computes velocity control voltage using interpolated kV from lookup table.
   * Uses the nonlinear kV lookup for better speed tracking across the full RPM range.
   *
   * @param targetRpm Target RPM (in 1/min)
   * @param actualRpm Actual RPM (in 1/min)
   * @param kP Proportional gain (V/RPS error)
   * @param kS Static friction compensation (volts)
   * @return Control voltage clamped to [-12, 12]
   */
  public static double computeVelocityVoltsWithLookup(
      double targetRpm, double actualRpm, double kP, double kS) {
    // Convert RPM to RPS for computation
    double targetRps = targetRpm / 60.0;
    double actualRps = actualRpm / 60.0;

    double kvInterp = getInterpolatedKV(targetRpm);
    double negErrorRps = Math.max(0.0, targetRps - actualRps);
    double volts = kvInterp * targetRps + kP * negErrorRps + kS;
    return MathUtil.clamp(volts, -kMaxVoltage, kMaxVoltage);
  }
}
