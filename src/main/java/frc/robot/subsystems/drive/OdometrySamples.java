package frc.robot.subsystems.drive;

/** Validates a complete common-timestamp frame before any module deltas are consumed. */
final class OdometrySamples {
  private OdometrySamples() {}

  static boolean aligned(
      int index,
      double timestamp,
      double[][] moduleTimes,
      int[] positionCounts,
      boolean useGyro,
      double[] gyroTimes,
      int gyroPositionCount) {
    if (!Double.isFinite(timestamp)) return false;
    for (int m = 0; m < moduleTimes.length; m++) {
      if (index >= moduleTimes[m].length
          || index >= positionCounts[m]
          || !sameTime(timestamp, moduleTimes[m][index])) return false;
    }
    return !useGyro
        || (index < gyroTimes.length
            && index < gyroPositionCount
            && sameTime(timestamp, gyroTimes[index]));
  }

  private static boolean sameTime(double a, double b) {
    return Double.isFinite(b) && Math.abs(a - b) <= 1e-5;
  }
}
