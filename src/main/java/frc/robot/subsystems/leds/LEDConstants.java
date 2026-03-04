package frc.robot.subsystems.leds;

import edu.wpi.first.wpilibj.util.Color;

/** Constants for the status LED strip (addressable LEDs on PWM). */
public final class LEDConstants {
  private LEDConstants() {}

  public static final int kPort = 9;
  public static final int kLength = 60;
  public static final Color kPurple = new Color(128, 0, 128);
  public static final Color kNavyBlue = new Color(0, 0, 80);
  public static final Color kGold = new Color(255, 200, 50);
}
