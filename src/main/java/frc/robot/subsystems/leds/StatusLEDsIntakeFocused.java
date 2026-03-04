package frc.robot.subsystems.leds;

import static edu.wpi.first.units.Units.Hertz;
import static edu.wpi.first.units.Units.Percent;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotState;

/**
 * Alternative LED subsystem implementation that focuses on intake state control.
 * 
 * This demonstrates LED control based on a single input subsystem (Intake).
 * Replace the standard StatusLEDs periodic() call with this implementation
 * to enable intake-only LED control.
 * 
 * Key states controlled:
 * - Intake deployed (down position): Blinking lime green
 * - Intake retracted (up position): Solid orange
 * - Intake moving: Blinking cyan (rapid)
 * - Robot disabled: Scrolling gradient (idle)
 * 
 * To use this instead of StatusLEDs, you would replace the intake in RobotContainer:
 * Instead of: led = new StatusLEDs();
 * Use: led = new StatusLEDsIntakeFocused();
 */
public class StatusLEDsIntakeFocused extends SubsystemBase {

  private final AddressableLED led;
  private final AddressableLEDBuffer buffer;
  private final RobotState state;

  // Intake-specific patterns
  private final LEDPattern patternIntakeDeployed;
  private final LEDPattern patternIntakeRetracted;
  private final LEDPattern patternIntakeMoving;
  private final LEDPattern patternIdle;

  public StatusLEDsIntakeFocused() {
    state = RobotState.getInstance();
    led = new AddressableLED(LEDConstants.kPort);
    buffer = new AddressableLEDBuffer(LEDConstants.kLength);
    led.setLength(LEDConstants.kLength);
    led.setData(buffer);
    led.start();

    // Pattern for deployed intake (down position)
    // Blinking lime green indicates active intake state
    patternIntakeDeployed = LEDPattern.solid(LEDConstants.kLimeGreen).blink(Seconds.of(0.5));

    // Pattern for retracted intake (up position)
    // Solid orange indicates safe/stowed state
    patternIntakeRetracted = LEDPattern.solid(LEDConstants.kOrange);

    // Pattern for moving intake (transitioning between states)
    // Rapid cyan blinking indicates active motion
    patternIntakeMoving = LEDPattern.solid(Color.kCyan).blink(Seconds.of(0.15));

    // Idle pattern when disabled
    // Scrolling gradient creates a visual feedback of robot status
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

  @Override
  public void periodic() {
    LEDPattern chosen = choosePatternByIntakeState();
    chosen.applyTo(buffer);
    led.setData(buffer);
  }

  /**
   * Chooses an LED pattern based solely on intake state.
   * This demonstrates LED state control for a single input (intake subsystem).
   * 
   * Decision tree:
   * 1. Robot is disabled → show idle pattern
   * 2. Intake is moving (intaking && !intakeDown) → show cyan blinking
   * 3. Intake is deployed (intakeDown == true) → show green blinking
   * 4. Intake is retracted (intakeDown == false) → show orange solid
   * 
   * @return LEDPattern to apply to the LED strip
   */
  private LEDPattern choosePatternByIntakeState() {
    if (DriverStation.isDisabled()) {
      return patternIdle;
    }

    // Get intake state from RobotState singleton
    boolean intakeDown = state.isIntakeDown();
    boolean isIntaking = state.isIntaking();

    // Detect if intake is in motion
    // isIntaking == true typically means the intake motor is actively running
    // Combined with !intakeDown, this suggests deployment in progress
    if (isIntaking && !intakeDown) {
      return patternIntakeMoving;
    }

    // If intake is at the deployed position (wrist angle >= 90 degrees)
    if (intakeDown) {
      return patternIntakeDeployed;
    } else {
      // Intake is retracted (safe position)
      return patternIntakeRetracted;
    }
  }
}
