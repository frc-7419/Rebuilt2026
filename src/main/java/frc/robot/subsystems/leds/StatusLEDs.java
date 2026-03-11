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
  private final LEDPattern patternShootingRpmInRange;
  private final LEDPattern patternShootingRpmSpinningUp;
  private final LEDPattern patternReadyToShoot;
  private final LEDPattern patternArcValidRpmSpinningUp;
  private final LEDPattern patternIntaking;
  private final LEDPattern patternIntakingAutoAimArcValid;
  private final LEDPattern patternShootingWhileIntaking;
  private final LEDPattern patternIdleRed;
  private final LEDPattern patternIdleBlue;

  public StatusLEDs() {
    state = RobotState.getInstance();
    led = new AddressableLED(LEDConstants.kPort);
    buffer = new AddressableLEDBuffer(LEDConstants.kLength);
    led.setLength(LEDConstants.kLength);
    led.setData(buffer);
    led.start();

    patternNotHubMode = LEDPattern.solid(LEDConstants.kPurple).blink(Seconds.of(0.5));
    patternNoAutoAimArc = LEDPattern.solid(Color.kRed);
    // RPM in range
    patternShootingRpmInRange =
        (r, w) -> flashBetween(w, Color.kGreen, LEDConstants.kBrightGreen, 0.2);
    // Spinning up
    patternShootingRpmSpinningUp = (r, w) -> flashBetween(w, Color.kGreen, Color.kYellow, 0.25);
    // Ready to shoot
    patternReadyToShoot = (r, w) -> flashBetween(w, LEDConstants.kBrightGreen, Color.kGreen, 0.4);
    // Arc valid but RPM still spinning up
    patternArcValidRpmSpinningUp = (r, w) -> flashBetween(w, Color.kGreen, Color.kYellow, 0.35);
    patternIntaking = LEDPattern.solid(Color.kYellow).blink(Seconds.of(0.25));
    patternIntakingAutoAimArcValid = (r, w) -> alternateYellowGreen(w, 0.25);
    // Shooting and intaking at once
    patternShootingWhileIntaking =
        (r, w) -> flashBetween(w, LEDConstants.kOrange, Color.kYellow, 0.2);

    patternIdleRed =
        LEDPattern.gradient(
                LEDPattern.GradientType.kContinuous, Color.kRed, Color.kWhite, Color.kRed)
            .scrollAtRelativeSpeed(Hertz.of(0.2))
            .breathe(Seconds.of(2.5))
            .atBrightness(Percent.of(100));
    patternIdleBlue =
        LEDPattern.gradient(
                LEDPattern.GradientType.kContinuous, Color.kBlue, Color.kWhite, Color.kBlue)
            .scrollAtRelativeSpeed(Hertz.of(0.2))
            .breathe(Seconds.of(2.5))
            .atBrightness(Percent.of(100));
  }

  private static void flashBetween(LEDWriter writer, Color a, Color b, double periodSec) {
    boolean useA = ((int) (Timer.getFPGATimestamp() / periodSec)) % 2 == 0;
    Color c = useA ? a : b;
    for (int i = 0; i < LEDConstants.kLength; i++) {
      writer.setLED(i, c);
    }
  }

  private void alternateYellowGreen(LEDWriter writer, double periodSec) {
    flashBetween(writer, Color.kYellow, Color.kGreen, periodSec);
  }

  @Override
  public void periodic() {
    LEDPattern chosen = choosePattern();
    chosen.applyTo(buffer);
    led.setData(buffer);
  }

  private LEDPattern choosePattern() {
    if (DriverStation.isDisabled()) {
      return state.isRedAlliance() ? patternIdleRed : patternIdleBlue;
    }
    if (!state.isHubMode() && state.isAutoAimEnabled()) {
      return patternNotHubMode;
    }
    if (!state.isAutoAimArcValid() && state.isAutoAimEnabled()) {
      return patternNoAutoAimArc;
    }
    // Shooting + intaking: distinct orange/yellow flash
    if (state.isShooting() && state.isIntaking()) {
      return patternShootingWhileIntaking;
    }
    // Shooting only: reflect RPM — in range = firing, else spinning up
    if (state.isShooting() && state.isAutoAimEnabled()) {
      return state.isShooterRpmInRange() ? patternShootingRpmInRange : patternShootingRpmSpinningUp;
    }
    // Intaking with valid arc (aiming while intaking)
    if (state.isAutoAimArcValid() && state.isIntaking() && state.isAutoAimEnabled()) {
      return patternIntakingAutoAimArcValid;
    }
    if (state.isIntaking()) {
      return patternIntaking;
    }
    // Arc valid, not shooting: show RPM state — ready to shoot vs spinning up
    if (state.isAutoAimArcValid() && state.isAutoAimEnabled()) {
      return state.isShooterRpmInRange() ? patternReadyToShoot : patternArcValidRpmSpinningUp;
    }

    return state.isRedAlliance() ? patternIdleRed : patternIdleBlue;
  }
}
