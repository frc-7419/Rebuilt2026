package frc.robot.subsystems.leds;

import static edu.wpi.first.units.Units.Hertz;
import static edu.wpi.first.units.Units.Percent;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.LEDWriter;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotState;

public class StatusLEDs extends SubsystemBase {

  private final AddressableLED led;
  private final AddressableLEDBuffer buffer;
  private final RobotState state;

  private final LEDPattern patternNotHubMode;
  private final LEDPattern patternNoAutoAimArc;
  private final LEDPattern patternShooting;
  private final LEDPattern patternAutoAimArcValid;
  private final LEDPattern patternIntaking;
  private final LEDPattern patternIdle;
  private final LEDPattern patternIntakingAutoAimArcValid;

  public StatusLEDs() {
    state = RobotState.getInstance();
    led = new AddressableLED(LEDConstants.kPort);
    buffer = new AddressableLEDBuffer(LEDConstants.kLength);
    led.setLength(LEDConstants.kLength);
    led.setData(buffer);
    led.start();

    patternNotHubMode = LEDPattern.solid(LEDConstants.kPurple).blink(Seconds.of(0.5));
    patternNoAutoAimArc = LEDPattern.solid(Color.kRed);
    patternShooting = LEDPattern.solid(Color.kGreen).blink(Seconds.of(0.25));
    patternAutoAimArcValid = LEDPattern.solid(Color.kGreen);
    patternIntaking = LEDPattern.solid(Color.kYellow).blink(Seconds.of(0.25));
    patternIntakingAutoAimArcValid = (r, w) -> alternateYellowGreen(w);

    patternIdle =
        LEDPattern.gradient(
                LEDPattern.GradientType.kContinuous,
                LEDConstants.kNavyBlue,
                LEDConstants.kGold,
                LEDConstants.kNavyBlue)
            .scrollAtRelativeSpeed(Hertz.of(0.2))
            .breathe(Seconds.of(2.5))
            .atBrightness(Percent.of(100));
  }

  private void alternateYellowGreen(LEDWriter writer) {
    boolean yellow = ((int) (Timer.getFPGATimestamp() / 0.25)) % 2 == 0;
    Color c = yellow ? Color.kYellow : Color.kGreen;
    for (int i = 0; i < LEDConstants.kLength; i++) {
      writer.setLED(i, c);
    }
  }

  @Override
  public void periodic() {
    LEDPattern chosen = choosePattern();
    chosen.applyTo(buffer);
    led.setData(buffer);
  }

  private LEDPattern choosePattern() {
    if (DriverStation.isDisabled()) {
      return patternIdle;
    }
    if (!state.isHubMode() && state.isAutoAimEnabled()) {
      return patternNotHubMode;
    }
    if (!state.isAutoAimArcValid() && state.isAutoAimEnabled()) {
      return patternNoAutoAimArc;
    }
    if (state.isAutoAimArcValid() && state.isShooting() && state.isAutoAimEnabled()) {
      return patternShooting;
    }
    if (state.isAutoAimArcValid() && state.isIntaking() && state.isAutoAimEnabled()) {
      return patternIntakingAutoAimArcValid;
    }
    if (state.isIntaking()) {
      return patternIntaking;
    }
    if (state.isAutoAimArcValid() && state.isAutoAimEnabled()) {
      return patternAutoAimArcValid;
    }

    return patternIdle;
  }
}
