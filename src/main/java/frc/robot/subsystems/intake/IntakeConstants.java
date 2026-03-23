package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Degrees;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;

public final class IntakeConstants {
  private IntakeConstants() {}

  // Motor IDs
  public static final int kIntakeWheelMotorLeftId = 25;
  public static final int kIntakeWheelMotorRightId = 26;
  public static final int kIntakeWristMotorId = 23;

  // Wrist configuration
  public static final Angle kMinWristAngle = Degrees.of(0.0);
  public static final Angle kMaxWristAngle = Degrees.of(120.0);
  public static final Angle kHomeWristAngle = Degrees.of(0.0);

  // Gear ratios
  // Intake wheel rollers: 2:1 front roller -> 4:3 back roller reduction
  public static final double kWheelMotorToWheelGearRatio = (2.0 / 1.0) * (4.0 / 3.0);
  // Intake pivot/wrist: 40:8 -> 40:20 -> 36:10 reductions
  public static final double kWristMotorToWristGearRatio =
      (40.0 / 8.0) * (40.0 / 20.0) * (36.0 / 10.0);

  // Voltage limits
  public static final double kMaxVoltage = 12.0;

  // Wheel motor configuration
  public static final TalonFXConfiguration wheelMotorConfig = new TalonFXConfiguration();
  public static final Slot0Configs wheelSlot0Configs = wheelMotorConfig.Slot0;

  static {
    wheelMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    wheelMotorConfig.CurrentLimits.StatorCurrentLimit = 80;
    wheelMotorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    wheelSlot0Configs.kP = 0.43;
    wheelSlot0Configs.kI = 0.0;
    wheelSlot0Configs.kD = 0.0;
    wheelSlot0Configs.kV = 0.0;
    wheelSlot0Configs.kS = 0.0;
  }

  // Wrist motor configuration
  public static final TalonFXConfiguration wristMotorConfig = new TalonFXConfiguration();
  public static final Slot0Configs wristSlot0Configs = wristMotorConfig.Slot0;

  static {
    wristMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    wristMotorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    wristMotorConfig.CurrentLimits.StatorCurrentLimit = 80;
    wristMotorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    wristSlot0Configs.kP = 2.0;
    wristSlot0Configs.kI = 0.0;
    wristSlot0Configs.kD = 0.3;
    wristSlot0Configs.kV = 0.0;
    wristSlot0Configs.kS = 0.0;
  }

  // Simulation PID gains
  public static final double kSimP = 10.0;
  public static final double kSimI = 0.0;
  public static final double kSimD = 0.0;
}
