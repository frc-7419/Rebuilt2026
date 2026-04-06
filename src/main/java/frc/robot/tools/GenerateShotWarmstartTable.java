package frc.robot.tools;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Constants;
import frc.robot.subsystems.shooter.ShooterConstants;
import frc.robot.util.KinematicsHelper;
import frc.robot.util.ShotWarmstartTable;
import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

/**
 * Offline tool: calls {@link KinematicsHelper#solveFunnelClearance} with the same geometry as hub
 * auto-aim and prints Java source for {@link ShotWarmstartTable} arrays.
 *
 * <p>Usage (Gradle): {@code ./gradlew generateShotWarmstartTable --args='--help'}
 *
 * <p>Examples:
 *
 * <ul>
 *   <li>{@code ./gradlew generateShotWarmstartTable} — default buckets & launch constant
 *   <li>{@code ./gradlew generateShotWarmstartTable --args='--launch-vel 0.19'}
 *   <li>{@code ./gradlew generateShotWarmstartTable --args='--buckets 0.5,1.0,...,6.0'} — override
 *       default buckets
 *   <li>{@code ./gradlew generateShotWarmstartTable --args='--pose-x 3 --pose-y 4 --pose-deg 0 --vx
 *       1 --vy 0 --tof 0.45'} — motion-corrected sample distance (bearing from pose to blue hub)
 * </ul>
 */
public final class GenerateShotWarmstartTable {

  private static final double kLaunchOffsetM = 0.28;

  private GenerateShotWarmstartTable() {}

  public static void main(String[] args) {
    if (hasFlag(args, "--help") || hasFlag(args, "-h")) {
      printHelp();
      return;
    }

    double launchVelConstant = ShooterConstants.kFuelLaunchVelMetersPerSecPerRotPerSec;
    double[] buckets = parseBuckets(getArg(args, "--buckets", null));
    if (buckets == null) {
      buckets = ShotWarmstartTable.DIST_BUCKETS.clone();
    }

    String lv = getArg(args, "--launch-vel", null);
    if (lv != null) {
      launchVelConstant = Double.parseDouble(lv);
    }

    double launchH = Constants.hoodBasePose.getZ();
    double targetZ = Constants.kHubTargetHeightMeters;
    double funnelR = Constants.kHubFunnelRadiusMeters;
    double funnelClear = Constants.kHubFunnelHeightMeters + Constants.kHubFunnelClearanceMeters;
    double fallbackMps = (ShooterConstants.kAutoAimRPM / 60.0) * launchVelConstant;

    Double poseX = parseOptionalDouble(getArg(args, "--pose-x", null));
    Double poseY = parseOptionalDouble(getArg(args, "--pose-y", null));
    Double poseDeg = parseOptionalDouble(getArg(args, "--pose-deg", null));
    Double vx = parseOptionalDouble(getArg(args, "--vx", null));
    Double vy = parseOptionalDouble(getArg(args, "--vy", null));
    String tofArg = getArg(args, "--tof", null);
    double nominalTof = tofArg != null ? Double.parseDouble(tofArg) : 0.45;

    Translation2d hub = Constants.kHubPoseBlue.getTranslation();

    List<Double> distCol = new ArrayList<>();
    List<Double> hoodCol = new ArrayList<>();
    List<Double> spdCol = new ArrayList<>();

    for (double dBucket : buckets) {
      double distForSolve = dBucket;
      if (poseX != null && poseY != null && poseDeg != null && vx != null && vy != null) {
        Pose2d pose = new Pose2d(poseX, poseY, Rotation2d.fromDegrees(poseDeg));
        Translation2d pivot = KinematicsHelper.getTurretPivotTranslation(pose);
        Translation2d toHub = hub.minus(pivot);
        double bearing = toHub.getAngle().getRadians();
        distForSolve =
            ShotWarmstartTable.effectiveDistForLookup(dBucket, bearing, vx, vy, nominalTof);
      } else if (vx != null && vy != null) {
        // No pose: crude +X motion component only
        distForSolve = Math.max(0.05, dBucket - vx * nominalTof);
      }

      var sol =
          KinematicsHelper.solveFunnelClearance(
              distForSolve, launchH, targetZ, funnelR, funnelClear, fallbackMps);
      distCol.add(dBucket);
      hoodCol.add(sol.hoodAngleRad());
      spdCol.add(sol.launchSpeedMps());
    }

    printJavaArrays(distCol, hoodCol, spdCol, launchVelConstant);

    if (poseX != null && poseY != null && poseDeg != null) {
      Pose2d pose = new Pose2d(poseX, poseY, Rotation2d.fromDegrees(poseDeg));
      Translation2d pivot = KinematicsHelper.getTurretPivotTranslation(pose);
      Translation2d toHub = hub.minus(pivot);
      double pivotToHub = toHub.getNorm();
      double distLaunch = Math.max(0.01, pivotToHub - kLaunchOffsetM);
      System.out.println();
      System.out.println("// --- pose check (blue hub) ---");
      System.out.printf(
          Locale.US, "// pivotToHubHoriz=%.4f m  distFromLaunch=%.4f m%n", pivotToHub, distLaunch);
    }
  }

  private static void printHelp() {
    System.out.println(
        "Generate ShotWarmstartTable Java literals via KinematicsHelper.solveFunnelClearance.");
    System.out.println();
    System.out.println("Options:");
    System.out.println("  --buckets 2,3,4,...   distance buckets (m), launch-horizontal frame");
    System.out.println("  --launch-vel <k>      Shooter launch vel constant (m/s per rot/s)");
    System.out.println(
        "  --vx --vy             optional field pivot velocity (m/s); adjusts sample distance");
    System.out.println("  --tof <s>             nominal TOF for motion correction (default 0.45)");
    System.out.println(
        "  --pose-x --pose-y --pose-deg   optional robot pose for comment-line range check");
    System.out.println("  -h, --help");
  }

  private static boolean hasFlag(String[] args, String flag) {
    for (String a : args) {
      if (flag.equals(a)) return true;
    }
    return false;
  }

  private static String getArg(String[] args, String key, String defaultVal) {
    for (int i = 0; i < args.length - 1; i++) {
      if (key.equals(args[i])) {
        return args[i + 1];
      }
    }
    return defaultVal;
  }

  private static Double parseOptionalDouble(String s) {
    if (s == null) return null;
    return Double.parseDouble(s);
  }

  private static double[] parseBuckets(String csv) {
    if (csv == null || csv.isBlank()) return null;
    String[] parts = csv.split(",");
    double[] out = new double[parts.length];
    for (int i = 0; i < parts.length; i++) {
      out[i] = Double.parseDouble(parts[i].trim());
    }
    return out;
  }

  private static void printJavaArrays(
      List<Double> distCol, List<Double> hoodCol, List<Double> spdCol, double launchVelUsed) {
    System.out.println("// Generated by GenerateShotWarmstartTable");
    System.out.printf(
        Locale.US,
        "// launchVelConstant=%.6f  kAutoAimRPM=%.1f%n",
        launchVelUsed,
        ShooterConstants.kAutoAimRPM);
    System.out.println(
        "// Paste into ShotWarmstartTable.java (replace DIST_BUCKETS, HOOD_GUESS_RAD, LAUNCH_SPEED_GUESS_MPS)");
    System.out.println();
    System.out.println("  public static final double[] DIST_BUCKETS = {");
    printDoubles(distCol, "    ", ",");
    System.out.println("  };");
    System.out.println();
    System.out.println("  public static final double[] HOOD_GUESS_RAD = {");
    printDoubles(hoodCol, "    ", ",");
    System.out.println("  };");
    System.out.println();
    System.out.println("  public static final double[] LAUNCH_SPEED_GUESS_MPS = {");
    printDoubles(spdCol, "    ", ",");
    System.out.println("  };");
  }

  private static void printDoubles(List<Double> vals, String indent, String sep) {
    StringBuilder sb = new StringBuilder(indent);
    for (int i = 0; i < vals.size(); i++) {
      if (i > 0) sb.append(sep).append(' ');
      sb.append(String.format(Locale.US, "%.6f", vals.get(i)));
    }
    System.out.println(sb);
  }
}
