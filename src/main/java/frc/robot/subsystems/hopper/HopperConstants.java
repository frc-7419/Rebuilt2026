package frc.robot.subsystems.hopper;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RPM;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;

public class HopperConstants {
  private HopperConstants() {}

  public static final Distance kHopperMaxExtension = Meters.of(0.301625);

  // Motor ID
  public static final int kHopperMotorId = 13;

  // Gear ratio (motor to serializer)
  public static final double kMotorToHopperGearRatio = 1.0;

  // Velocity limits
  public static final AngularVelocity kMinVelocity = RPM.of(0.0);
  public static final AngularVelocity kMaxVelocity = RPM.of(5000.0);

  // Motor configuration
  public static final TalonFXConfiguration motorConfig = new TalonFXConfiguration();
  public static final Slot0Configs motorSlot0Configs = motorConfig.Slot0;

  static {
    motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    motorSlot0Configs.kP = 5.0;
    motorSlot0Configs.kI = 0.0;
    motorSlot0Configs.kD = 0.05;
    motorSlot0Configs.kV = 0.0;
    motorSlot0Configs.kS = 0.0;
  }

  // Simulation PID gains
  public static final double kSimP = 1.0;
  public static final double kSimI = 0.0;
  public static final double kSimD = 0.0;
}
