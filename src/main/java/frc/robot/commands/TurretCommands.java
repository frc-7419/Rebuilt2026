package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import frc.robot.subsystems.turret.Turret;
import java.text.DecimalFormat;
import java.text.NumberFormat;
import java.util.LinkedList;
import java.util.List;
import java.util.concurrent.atomic.AtomicBoolean;

public class TurretCommands {
  private static final double FF_START_DELAY = 2.0; // Secs
  private static final double FF_RAMP_RATE = 0.1; // Volts/Sec
  private static final double FF_SEEK_VOLTS = 1.25;
  private static final double FF_SEEK_TIMEOUT_SEC = 60.0;

  private TurretCommands() {}

  /**
   * Measures velocity feedforward (kS, kV) for the turret using a linear voltage ramp. Same method
   * as {@link DriveCommands#feedforwardCharacterization}; velocity is in mechanism rotations per
   * second to match Phoenix Slot0 kV units.
   *
   * <p>First drives slowly to the minimum soft limit, then settles, then ramps voltage positive so
   * the sweep uses as much travel as possible toward the max limit. Voltage is clamped near limits
   * so the turret does not command into the stops. Cancel when finished (e.g. disable robot or
   * switch auto).
   */
  public static Command feedforwardCharacterization(Turret turret) {
    List<Double> velocitySamples = new LinkedList<>();
    List<Double> voltageSamples = new LinkedList<>();
    Timer timer = new Timer();
    AtomicBoolean seekOk = new AtomicBoolean(false);

    Command rampPhase =
        Commands.sequence(
            Commands.run(() -> turret.runCharacterization(0.0), turret).withTimeout(FF_START_DELAY),
            Commands.runOnce(timer::restart),
            Commands.run(
                    () -> {
                      double voltage = timer.get() * FF_RAMP_RATE;
                      turret.runCharacterization(voltage);
                      velocitySamples.add(turret.getFFCharacterizationVelocity());
                      voltageSamples.add(voltage);
                    },
                    turret)
                .finallyDo(
                    (interrupted) -> {
                      turret.stop();
                      int n = velocitySamples.size();
                      if (n < 2) {
                        System.out.println(
                            "Turret FF: not enough samples for regression (cancel early or no"
                                + " motion).");
                        return;
                      }
                      double sumX = 0.0;
                      double sumY = 0.0;
                      double sumXY = 0.0;
                      double sumX2 = 0.0;
                      for (int i = 0; i < n; i++) {
                        sumX += velocitySamples.get(i);
                        sumY += voltageSamples.get(i);
                        sumXY += velocitySamples.get(i) * voltageSamples.get(i);
                        sumX2 += velocitySamples.get(i) * velocitySamples.get(i);
                      }
                      double denom = n * sumX2 - sumX * sumX;
                      if (Math.abs(denom) < 1e-9) {
                        System.out.println("Turret FF: regression singular (zero velocity span).");
                        return;
                      }
                      double kS = (sumY * sumX2 - sumX * sumXY) / denom;
                      double kV = (n * sumXY - sumX * sumY) / denom;

                      NumberFormat formatter = new DecimalFormat("#0.00000");
                      System.out.println(
                          "********** Turret FF Characterization Results **********");
                      System.out.println(
                          "\t(velocity in rot/s mechanism — use with Phoenix Slot0 kV / kS)");
                      System.out.println("\tkS: " + formatter.format(kS));
                      System.out.println("\tkV: " + formatter.format(kV));
                    }));

    return Commands.sequence(
        Commands.runOnce(
            () -> {
              velocitySamples.clear();
              voltageSamples.clear();
            }),
        Commands.runOnce(
            () ->
                System.out.println(
                    "Turret FF: seeking to min limit (~"
                        + FF_SEEK_VOLTS
                        + " V), then ramping positive...")),
        Commands.run(() -> turret.runSeekToMinForCharacterization(FF_SEEK_VOLTS), turret)
            .until(turret::isNearMinLimitForCharacterization)
            .withTimeout(FF_SEEK_TIMEOUT_SEC)
            .finallyDo(
                (interrupted) -> {
                  boolean atMin = turret.isNearMinLimitForCharacterization();
                  seekOk.set(!interrupted && atMin);
                  if (!seekOk.get()) {
                    System.out.println(
                        interrupted
                            ? "Turret FF: seek interrupted or timed out — skipping ramp."
                            : "Turret FF: seek ended but not at min limit — skipping ramp.");
                  }
                  turret.stop();
                }),
        new ConditionalCommand(rampPhase, Commands.none(), seekOk::get));
  }
}
