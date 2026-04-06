package frc.robot.util;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

/**
 * Precomputed hub-shot warmstarts (hood angle, launch speed) vs horizontal distance from launch to
 * target, in the same frame as {@link KinematicsHelper#solveFunnelClearance}. Regenerate with
 * {@code ./gradlew generateShotWarmstartTable}.
 */
public final class ShotWarmstartTable {

  /**
   * Distance along the shot (m), horizontal from launch point to target — matches {@code distM}
   * passed to {@link KinematicsHelper#solveFunnelClearance} (e.g. pivot distance minus launch
   * offset in {@link frc.robot.commands.AutoAim}).
   */
  public static final double[] DIST_BUCKETS = {
    0.5, 1.0, 1.5, 2.0, 2.5, 3.0, 3.5, 4.0, 4.5, 5.0, 5.5, 6.0
  };

  /**
   * From {@code ./gradlew generateShotWarmstartTable} (hub geometry + existing table warm-start).
   */
  public static final double[] HOOD_GUESS_RAD = {
    1.308997, 1.308997, 1.307596, 1.242112, 1.192364, 1.154971, 1.125826, 1.102476, 1.083352,
    1.067407, 1.053911, 1.042341,
  };

  public static final double[] LAUNCH_SPEED_GUESS_MPS = {
    4.711562, 5.216685, 5.975996, 6.183640, 6.480253, 6.798129, 7.118714, 7.435110, 7.744594,
    8.046175, 8.339641, 8.625140,
  };

  private static final InterpolatingDoubleTreeMap kHoodByDistM = new InterpolatingDoubleTreeMap();
  private static final InterpolatingDoubleTreeMap kSpeedByDistM = new InterpolatingDoubleTreeMap();

  static {
    int n = DIST_BUCKETS.length;
    if (n != HOOD_GUESS_RAD.length || n != LAUNCH_SPEED_GUESS_MPS.length) {
      throw new AssertionError("ShotWarmstartTable: bucket arrays must have equal length");
    }
    for (int i = 0; i < n; i++) {
      kHoodByDistM.put(DIST_BUCKETS[i], HOOD_GUESS_RAD[i]);
      kSpeedByDistM.put(DIST_BUCKETS[i], LAUNCH_SPEED_GUESS_MPS[i]);
    }
  }

  /** Where {@code distFromLaunchM} sits relative to {@link #DIST_BUCKETS} for logging / tuning. */
  public enum DistanceRegion {
    INTERIOR,
    BELOW_MIN,
    ABOVE_MAX
  }

  public static DistanceRegion distanceRegion(double distFromLaunchM) {
    if (DIST_BUCKETS.length == 0) {
      return DistanceRegion.INTERIOR;
    }
    double min = DIST_BUCKETS[0];
    double max = DIST_BUCKETS[DIST_BUCKETS.length - 1];
    if (distFromLaunchM < min) {
      return DistanceRegion.BELOW_MIN;
    }
    if (distFromLaunchM > max) {
      return DistanceRegion.ABOVE_MAX;
    }
    return DistanceRegion.INTERIOR;
  }

  private ShotWarmstartTable() {}

  /**
   * First-order distance correction for motion: shrink lookup distance when moving toward the
   * target along the shot bearing (field frame).
   *
   * @param distLaunchHorizontalM horizontal launch-to-target distance (m)
   * @param bearingToTargetRad field bearing from launch/pivot toward target
   * @param pivotVxField pivot velocity X (field, m/s)
   * @param pivotVyField pivot velocity Y (field, m/s)
   * @param nominalTofSec rough time-of-flight for the correction (e.g. previous cycle TOF or ~0.45)
   */
  public static double effectiveDistForLookup(
      double distLaunchHorizontalM,
      double bearingToTargetRad,
      double pivotVxField,
      double pivotVyField,
      double nominalTofSec) {
    double c = Math.cos(bearingToTargetRad);
    double s = Math.sin(bearingToTargetRad);
    double toward = pivotVxField * c + pivotVyField * s;
    return Math.max(0.05, distLaunchHorizontalM - toward * nominalTofSec);
  }

  /** Interpolated { hoodRad, launchSpeedMps } via {@link InterpolatingDoubleTreeMap}. */
  public static double[] getInitialGuess(double distM) {
    if (DIST_BUCKETS.length == 0) {
      return new double[] {0.7, 18.0};
    }
    Double hood = kHoodByDistM.get(distM);
    Double spd = kSpeedByDistM.get(distM);
    if (hood == null || spd == null) {
      return new double[] {0.7, 18.0};
    }
    return new double[] {hood, spd};
  }
}
