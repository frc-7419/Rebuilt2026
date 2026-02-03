package frc.robot.subsystems.turret;

import static edu.wpi.first.units.Units.Rotations;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;

/** Basic turret configuration values. Update IDs and gains for your robot. */
public final class TurretConstants {
  private TurretConstants() {}

  // Default CAN ID for turret motor (override to match hardware)
  public static final int kTurretMotorId = 9;
  public static final int kEncoderOneId = 6;
  public static final int kEncoderTwoId = 7;

  // Control gains (software PID for safety). Tune as needed.
  public static final double kP = 10;
  public static final double kI = 0.0;
  public static final double kD = 1;

  // Maximum output voltage when using software PID (Volts)
  public static final double kMaxVoltage = 12.0;

  // Maximum rotation range in degreess
  public static final double kTurretMaxRotations = 720;

  // Allowed motion limits (radians).
  public static final double kMinAngleRad = Units.degreesToRadians(-kTurretMaxRotations / 2.0);
  public static final double kMaxAngleRad = Units.degreesToRadians(kTurretMaxRotations / 2.0);

  // Small deadband for joystick control
  public static final double kDeadband = 0.05;

  // Arbitary gear ratios
  public static final double kMotorToEncoderOneGearRatio = (52.0 / 16.0);
  public static final double kEncoderOneToEncoderTwoGearRatio = (9.0 / 1.0);
  public static final double kEncoderTwoToTurretGearRatio = (20.0 / 12.0);

  public static final double kMotorToEncoderTwoGearRatio =
      kMotorToEncoderOneGearRatio * kEncoderOneToEncoderTwoGearRatio;
  public static final double kMotorToTurretGearRatio =
      kMotorToEncoderTwoGearRatio * kEncoderTwoToTurretGearRatio;

  public static final Angle encoderOneZeroOffset = Rotations.of(0.2391);
  public static final Angle encoderTwoZeroOffset = Rotations.of(0.8421);

  // Turret pivot point offset from robot center (forward, left)
  // Positive forward = forward of robot center, positive left = left of robot center
  public static final Transform2d kTurretOffset =
      new Transform2d(new Translation2d(0.0, 0.0), new Rotation2d());
}
