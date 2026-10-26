package frc.robot.subsystems.leds;

import edu.wpi.first.wpilibj.util.Color;

/** Constants for the status LED strip (addressable LEDs on PWM). */
public final class LEDConstants {
  private LEDConstants() {}

  public static final int kPort = 9;
  public static final int kLength = 64;
  public static final Color kPurple = new Color(128, 0, 128);
  public static final Color kNavyBlue = new Color(0, 0, 80);
  public static final Color kGold = new Color(255, 200, 50);
  public static final Color kOrange = new Color(255, 120, 0);
  public static final Color kBrightGreen = new Color(0, 255, 80);

  public static final double kIntakeStallMinAbsRpm = 120.0;
  public static final double kIndexerStallMinAbsRpm = 100.0;
  public static final double kMechanismCommandVolts = 0.85;
  public static final double kMechanismStallDebounceSec = 0.35;
}
