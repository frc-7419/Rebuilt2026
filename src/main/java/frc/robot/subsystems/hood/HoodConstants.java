package frc.robot.subsystems.hood;

import static edu.wpi.first.units.Units.Degrees;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;

public final class HoodConstants {
  private HoodConstants() {}

  public static final int kHoodMotorId = 22;

  public static final double kMaxVoltage = 12.0;
  public static final double kDeadband = 0.05;

  public static final double kMotorToHoodGearRatio = (50.0 / 8.0) * (144.0 / 10.0);

  public static final Angle kMinAngle = Degrees.of(30.0);
  public static final Angle kMaxAngle = Degrees.of(64.0);

  public static final TalonFXConfiguration motorConfig = new TalonFXConfiguration();
  public static final Slot0Configs motorSlot0Configs = motorConfig.Slot0;
  public static final MotionMagicConfigs motionMagicConfigs = motorConfig.MotionMagic;

  static {
    motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    motorConfig.CurrentLimits.StatorCurrentLimit = 50;

    motorSlot0Configs.kP = 10.0;
    motorSlot0Configs.kI = 0.0;
    motorSlot0Configs.kD = 0.0;
    motorSlot0Configs.kV = 0.0;
    motorSlot0Configs.kS = 0.0;

    double cruiseRotPerSec = 0.5;
    motionMagicConfigs.MotionMagicCruiseVelocity = cruiseRotPerSec * kMotorToHoodGearRatio;
    motionMagicConfigs.MotionMagicAcceleration = motionMagicConfigs.MotionMagicCruiseVelocity / 0.2;
  }

  public static final double kSimP = 20.0;
  public static final double kSimI = 0.0;
  public static final double kSimD = 0.5;
}
