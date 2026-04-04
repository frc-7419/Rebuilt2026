package frc.robot.subsystems.serializer;

import static edu.wpi.first.units.Units.RPM;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.AngularVelocity;

public class SerializerConstants {
  private SerializerConstants() {}
  // Motor IDs
  public static final int kSerializerMotorId = 35;
  public static final int kFeederMotorId = 30;

  // Gear ratios
  // Serializer wheel: 3:1 -> 40:20 reductions (total 6.0x)
  public static final double kMotorToSerializerGearRatio = 3.0 * (40.0 / 20.0);
  // Feeder rollers: 24:12 reduction (2.0x)
  public static final double kMotorToFeederGearRatio = (24.0 / 12.0);

  // Velocity limits
  public static final AngularVelocity kMinVelocity = RPM.of(0.0);
  public static final AngularVelocity kMaxVelocity = RPM.of(5000.0);

  // Serializer wheel motor configuration
  public static final TalonFXConfiguration serializerMotorConfig = new TalonFXConfiguration();
  public static final Slot0Configs serializerSlot0Configs = serializerMotorConfig.Slot0;

  // Feeder rollers motor configuration
  public static final TalonFXConfiguration feederMotorConfig = new TalonFXConfiguration();
  public static final Slot0Configs feederSlot0Configs = feederMotorConfig.Slot0;

  static {
    serializerMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    serializerSlot0Configs.kP = 5.0;
    serializerSlot0Configs.kI = 0.0;
    serializerSlot0Configs.kD = 0.05;
    serializerSlot0Configs.kV = 0.0;
    serializerSlot0Configs.kS = 0.0;
    serializerMotorConfig.CurrentLimits.StatorCurrentLimit = 80.0;
    serializerMotorConfig.CurrentLimits.StatorCurrentLimitEnable = true;

    feederMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    feederMotorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    feederSlot0Configs.kP = 5.0;
    feederSlot0Configs.kI = 0.0;
    feederSlot0Configs.kD = 0.05;
    feederSlot0Configs.kV = 0.0;
    feederSlot0Configs.kS = 0.0;
    feederMotorConfig.CurrentLimits.StatorCurrentLimit = 80.0;
    feederMotorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
  }

  // Simulation PID gains
  public static final double kSimP = 1.0;
  public static final double kSimI = 0.0;
  public static final double kSimD = 0.0;
}
