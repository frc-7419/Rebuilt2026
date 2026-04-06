package frc.robot.subsystems.leds;

import static edu.wpi.first.units.Units.Hertz;
import static edu.wpi.first.units.Units.Percent;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.LEDWriter;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotState;
import org.littletonrobotics.junction.Logger;

public class StatusLEDs extends SubsystemBase {

  private final AddressableLED led;
  private final AddressableLEDBuffer buffer;
  private final RobotState state;

  private final LEDPattern patternMechanismFault;
  private final LEDPattern patternShootingRpmInRange;
  private final LEDPattern patternShootingRpmSpinningUp;
  private final LEDPattern patternReadyToShoot;
  private final LEDPattern patternAutoAimSpinningUp;
  private final LEDPattern patternIntaking;
  private final LEDPattern patternShootingWhileIntaking;
  private final LEDPattern patternIdleRed;
  private final LEDPattern patternIdleBlue;

  private double intakeStallSince = Double.NaN;
  private double indexerStallSince = Double.NaN;

  public StatusLEDs() {
    state = RobotState.getInstance();
    led = new AddressableLED(LEDConstants.kPort);
    buffer = new AddressableLEDBuffer(LEDConstants.kLength);
    led.setLength(LEDConstants.kLength);
    led.setData(buffer);
    led.start();

    patternMechanismFault = (r, w) -> flashBetween(w, Color.kRed, Color.kBlack, 0.12);
    // RPM in range
    patternShootingRpmInRange =
        (r, w) -> flashBetween(w, Color.kGreen, LEDConstants.kBrightGreen, 0.2);
    // Spinning up
    patternShootingRpmSpinningUp = (r, w) -> flashBetween(w, Color.kGreen, Color.kYellow, 0.25);
    // Ready to shoot
    patternReadyToShoot = (r, w) -> flashBetween(w, LEDConstants.kBrightGreen, Color.kGreen, 0.4);
    patternAutoAimSpinningUp = (r, w) -> flashBetween(w, Color.kGreen, Color.kYellow, 0.35);
    patternIntaking = LEDPattern.solid(Color.kYellow).blink(Seconds.of(0.12));
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

  @Override
  public void periodic() {
    refreshMechanismFaultDebounces();
    LEDPattern chosen = choosePattern();
    chosen.applyTo(buffer);
    led.setData(buffer);

    Logger.recordOutput(
        "StatusLEDs/DisplayedColorRGB",
        new double[] {
          buffer.getRed(0), buffer.getGreen(0), buffer.getBlue(0),
        });
  }

  private void refreshMechanismFaultDebounces() {
    double now = Timer.getFPGATimestamp();

    if (!intakeStallRaw()) {
      intakeStallSince = Double.NaN;
    } else if (Double.isNaN(intakeStallSince)) {
      intakeStallSince = now;
    }

    if (!indexerStallRaw()) {
      indexerStallSince = Double.NaN;
    } else if (Double.isNaN(indexerStallSince)) {
      indexerStallSince = now;
    }
  }

  private boolean intakeStallRaw() {
    return state.isIntaking()
        && Math.abs(state.getIntakeWheelAppliedVolts()) >= LEDConstants.kMechanismCommandVolts
        && state.isIntakeWheelDeviceConnected()
        && Math.abs(state.getLatestIntakeWheelVelocity().in(RPM))
            < LEDConstants.kIntakeStallMinAbsRpm;
  }

  private boolean indexerStallRaw() {
    double serV = state.getSerializerAppliedVolts();
    double feedV = state.getFeederAppliedVolts();
    double serRpm = Math.abs(state.getLatestHopperVelocity().in(RPM));
    double feedRpm = Math.abs(state.getLatestFeederVelocity().in(RPM));
    boolean serCmd = Math.abs(serV) >= LEDConstants.kMechanismCommandVolts;
    boolean feedCmd = Math.abs(feedV) >= LEDConstants.kMechanismCommandVolts;
    return (state.isSerializerDeviceConnected()
            && serCmd
            && serRpm < LEDConstants.kIndexerStallMinAbsRpm)
        || (state.isFeederDeviceConnected()
            && feedCmd
            && feedRpm < LEDConstants.kIndexerStallMinAbsRpm);
  }

  private boolean intakeStallDebounced() {
    double now = Timer.getFPGATimestamp();
    return !Double.isNaN(intakeStallSince)
        && now - intakeStallSince >= LEDConstants.kMechanismStallDebounceSec;
  }

  private boolean indexerStallDebounced() {
    double now = Timer.getFPGATimestamp();
    return !Double.isNaN(indexerStallSince)
        && now - indexerStallSince >= LEDConstants.kMechanismStallDebounceSec;
  }

  private boolean mechanismDisconnect() {
    if (Constants.currentMode == Constants.Mode.SIM) {
      return false;
    }
    return state.isAnySubsystemDeviceDisconnected();
  }

  private LEDPattern choosePattern() {
    if (DriverStation.isDisabled()) {
      return state.isRedAlliance() ? patternIdleRed : patternIdleBlue;
    }
    if (mechanismDisconnect() || intakeStallDebounced() || indexerStallDebounced()) {
      return patternMechanismFault;
    }
    // Shooting + intaking: distinct orange/yellow flash
    if (state.isShooting() && state.isIntaking()) {
      return patternShootingWhileIntaking;
    }
    // Shooting only: reflect RPM — in range = firing, else spinning up
    if (state.isShooting() && state.isAutoAimEnabled()) {
      return state.isShooterRpmInRange() ? patternShootingRpmInRange : patternShootingRpmSpinningUp;
    }
    if (state.isIntaking()) {
      return patternIntaking;
    }
    // Auto-aim enabled, idle: show shooter readiness (RPM) without arc-valid gating
    if (state.isAutoAimEnabled()) {
      return state.isShooterRpmInRange() ? patternReadyToShoot : patternAutoAimSpinningUp;
    }

    return state.isRedAlliance() ? patternIdleRed : patternIdleBlue;
  }
}
