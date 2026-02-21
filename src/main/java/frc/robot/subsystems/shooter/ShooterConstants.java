package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RPM;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Translation3d;
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
    motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    motorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    motorSlot0Configs.kP = 10;
    motorSlot0Configs.kI = 0;
    motorSlot0Configs.kD = 0.1;
    motorSlot0Configs.kV = 0;
    motorSlot0Configs.kS = 0;
  }

  public static final double kSimP = 1.0;
  public static final double kSimI = 0.0;
  public static final double kSimD = 0.0;

  public static final Angle kHoodZeroed = Degrees.of(0.0);

  public static final Translation3d kRobotToShooterRelease = new Translation3d(0.0, 0.0, 0.0);

  public static final Distance kShooterWheelRadius = Meters.of(0.050);

  public static final double kFuelLaunchVelMetersPerSecPerRotPerSec = 10.0;

  public static final double kSpinTransfer = 0.5;
}
