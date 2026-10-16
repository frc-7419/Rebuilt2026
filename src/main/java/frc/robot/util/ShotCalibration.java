package frc.robot.util;

import static edu.wpi.first.units.Units.Radians;

import frc.robot.subsystems.hood.HoodConstants;
import frc.robot.subsystems.shooter.ShooterConstants;
import java.util.List;
import java.util.Optional;

/** Complete, measured hub-shot settings. Distances use the existing launch-to-target convention. */
public final class ShotCalibration {
  public record Point(double distanceM, double hoodRad, double wheelRpm, double flightTimeSec) {}

  // Populate from practice measurements, in strictly increasing distance order. Never seed with
  // another robot's settings or the warm-start guesses, which are not measured flight times.
  public static final ShotCalibration MEASURED = new ShotCalibration(List.of());

  private final List<Point> points;

  public ShotCalibration(List<Point> points) {
    this.points = List.copyOf(points);
    double previous = 0.0;
    for (Point p : this.points) {
      if (!Double.isFinite(p.distanceM())
          || p.distanceM() <= previous
          || !Double.isFinite(p.hoodRad())
          || p.hoodRad() < HoodConstants.kMinAngle.in(Radians)
          || p.hoodRad() > HoodConstants.kMaxAngle.in(Radians)
          || !Double.isFinite(p.wheelRpm())
          || p.wheelRpm() < ShooterConstants.kAutoAimRPMMin
          || p.wheelRpm() > ShooterConstants.kAutoAimRPMMax
          || !Double.isFinite(p.flightTimeSec())
          || p.flightTimeSec() <= 0.0) {
        throw new IllegalArgumentException("Invalid or unordered measured shot calibration");
      }
      previous = p.distanceM();
    }
  }

  public Optional<Point> sample(double distanceM) {
    if (points.size() < 2
        || !Double.isFinite(distanceM)
        || distanceM < points.get(0).distanceM()
        || distanceM > points.get(points.size() - 1).distanceM()) return Optional.empty();
    for (int i = 1; i < points.size(); i++) {
      Point a = points.get(i - 1);
      Point b = points.get(i);
      if (distanceM <= b.distanceM()) {
        double t = (distanceM - a.distanceM()) / (b.distanceM() - a.distanceM());
        return Optional.of(
            new Point(
                distanceM,
                a.hoodRad() + t * (b.hoodRad() - a.hoodRad()),
                a.wheelRpm() + t * (b.wheelRpm() - a.wheelRpm()),
                a.flightTimeSec() + t * (b.flightTimeSec() - a.flightTimeSec())));
      }
    }
    return Optional.empty();
  }
}
