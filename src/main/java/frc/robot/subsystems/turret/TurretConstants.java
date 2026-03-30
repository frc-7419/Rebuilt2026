package frc.robot.subsystems.turret;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Angle;

/** Basic turret configuration values. Update IDs and gains for your robot. */
public final class TurretConstants {
  private TurretConstants() {}

  // Arbitary CAN IDs
  public static final int kTurretMotorId = 31;

  // Maximum output voltage when using software PID (Volts)
  public static final double kMaxVoltage = 12.0;

  // Maximum rotation range in degreess
  public static final double kAbsoluteTurretMaxRotations = 600;

  /**
   * The absolute min and max angles are the physical limits of the turret, which is set based on
   * the mechanical constraints of the robot.
   *
   * <p>The software min and max angles are the limits that the software will enforce, as the full
   * physical range is larger than what is desired for operation. This is set at 360 degrees, using
   * wraparound.
   */

  // Theoretical min and max angles
  public static final Angle kAbsoluteMinAngle = Degrees.of(-kAbsoluteTurretMaxRotations / 2.0);

  public static final Angle kAbsoluteMaxAngle = Degrees.of(kAbsoluteTurretMaxRotations / 2.0);

  // Software rotation ranges
  public static final double kTurretMaxRotations = 360;

  // Software min and max angles
  public static final Angle kMinAngle = Degrees.of(-kTurretMaxRotations / 2.0);
  public static final Angle kMaxAngle = Degrees.of(kTurretMaxRotations / 2.0);

  // Small deadband for joystick control
  public static final double kDeadband = 0.05;

  public static final double kMotorToRightEncoderGearRatio =
      (60.0 / 12.0) * (30.0 / 60.0) * (17.0 / 30.0);
  public static final double kMotorToLeftEncoderGearRatio =
      (60.0 / 12.0) * (30.0 / 60.0) * (16.0 / 30.0);
  public static final double kMotorToTurretGearRatio = (60.0 / 12.0) * (130.0 / 10.0);

  public static final double kRightEncoderToTurretGearRatio =
      (130.0 / 10.0) * (60.0 / 30.0) * (30.0 / 17.0);

  public static final double kLeftEncoderToTurretGearRatio =
      (130.0 / 10.0) * (60.0 / 30.0) * (30.0 / 16.0);

  public static final int kEncoderRightId = 6;
  public static final int kEncoderLeftId = 7;

  public static final Angle rightEncoderZeroOffset = Rotations.of(-0.092041);
  public static final Angle leftEncoderZeroOffset = Rotations.of(-0.357666);

  public static final TalonFXConfiguration motorConfig = new TalonFXConfiguration();
  public static final Slot0Configs motorSlot0Configs = motorConfig.Slot0;
  public static final FeedbackConfigs motorFeedbackConfigs = motorConfig.Feedback;

  public static final CANcoderConfiguration cancoderConfig = new CANcoderConfiguration();

  static {
    motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    motorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    motorConfig.CurrentLimits.StatorCurrentLimit = 30;
    motorSlot0Configs.kP = 300;
    motorSlot0Configs.kI = 0;
    motorSlot0Configs.kD = 10.2;
    motorSlot0Configs.kV = 8.99;
    motorSlot0Configs.kS = 0.44;
    motorFeedbackConfigs.RotorToSensorRatio = kMotorToRightEncoderGearRatio;
    motorFeedbackConfigs.SensorToMechanismRatio = kRightEncoderToTurretGearRatio;
    motorConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = kAbsoluteMaxAngle.in(Rotations);
    motorConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = kAbsoluteMinAngle.in(Rotations);
    motorConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    motorConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    motorFeedbackConfigs.FeedbackRemoteSensorID = 6;
    motorFeedbackConfigs.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;

    cancoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
    cancoderConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 1;
    cancoderConfig.MagnetSensor.MagnetOffset = 0;
  }

  public static final double kSimP = 20.0;
  public static final double kSimI = 0.0;
  public static final double kSimD = 0.1;

  // Turret pivot point offset from robot center (forward, left)
  // Positive forward = forward of robot center, positive left = left of robot center
  public static final Transform2d kTurretOffset =
      new Transform2d(new Translation2d(0.0, 0.0), new Rotation2d());
}
