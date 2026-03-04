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
  private final LEDPattern patternNoAutoAim;
  
  // Intake-specific patterns
  private final LEDPattern patternIntakeDeployed;
  private final LEDPattern patternIntakeRetracted;
  private final LEDPattern patternIntakeMoving;

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
    patternNoAutoAim = LEDPattern.solid(Color.kRed).blink(Seconds.of(0.5));

    patternIdle =
        LEDPattern.gradient(
                LEDPattern.GradientType.kContinuous,
                LEDConstants.kNavyBlue,
                LEDConstants.kGold,
                LEDConstants.kNavyBlue)
            .scrollAtRelativeSpeed(Hertz.of(0.2))
            .breathe(Seconds.of(2.5))
            .atBrightness(Percent.of(100));
    
    // Intake state patterns
    patternIntakeDeployed = LEDPattern.solid(LEDConstants.kLimeGreen).blink(Seconds.of(0.5));
    patternIntakeRetracted = LEDPattern.solid(LEDConstants.kOrange);
    patternIntakeMoving = LEDPattern.solid(Color.kCyan).blink(Seconds.of(0.15));
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

  /**
   * Chooses an LED pattern based on intake state.
   * This demonstrates LED state control for a single input (intake subsystem).
   * This method is provided as an example implementation.
   * To use this instead of the default choosePattern(), swap the calls in periodic().
   * 
   * Priority:
   * 1. Intake is moving (animating deployment/retraction)
   * 2. Intake is deployed/down
   * 3. Intake is retracted/up
   * 4. Idle pattern (default)
   */
  @SuppressWarnings("unused")
  private LEDPattern choosePatternByIntakeState() {
    if (DriverStation.isDisabled()) {
      return patternIdle;
    }
    
    // Get intake state from RobotState
    boolean intakeDown = state.isIntakeDown();
    boolean isIntaking = state.isIntaking();
    
    // Check if intake is actively moving (blinking cyan to indicate motion)
    // This could be enhanced with intake velocity feedback
    if (isIntaking && !intakeDown) {
      // Intake is in motion, likely deploying
      return patternIntakeMoving;
    }
    
    if (intakeDown) {
      // Intake is deployed, show solid blink pattern
      return patternIntakeDeployed;
    } else {
      // Intake is retracted, show solid orange
      return patternIntakeRetracted;
    }
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
    if (!state.isAutoAimEnabled()) {
      return patternNoAutoAim;
    }

    return patternIdle;
  }
}
