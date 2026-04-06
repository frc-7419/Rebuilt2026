package frc.robot.tools;

import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Constants;
import frc.robot.subsystems.hood.HoodConstants;
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
 *   <li>{@code ./gradlew generateShotWarmstartTable} — default distance buckets
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

    double[] buckets = parseBuckets(getArg(args, "--buckets", null));
    if (buckets == null) {
      buckets = ShotWarmstartTable.DIST_BUCKETS.clone();
    }

    double launchH = Constants.hoodBasePose.getZ();
    double targetZ = Constants.kHubTargetHeightMeters;
    double funnelR = Constants.kHubFunnelRadiusMeters;
    double funnelClear = Constants.kHubFunnelHeightMeters + Constants.kHubFunnelClearanceMeters;

    printConstantsUsed(launchH, targetZ, funnelR, funnelClear);

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

      // fallbackLaunchSpeedMps is unused inside KinematicsHelper.solveFunnelClearance today.
      var sol =
          KinematicsHelper.solveFunnelClearance(
              distForSolve, launchH, targetZ, funnelR, funnelClear, 0.0);
      distCol.add(dBucket);
      hoodCol.add(sol.hoodAngleRad());
      spdCol.add(sol.launchSpeedMps());
    }

    printJavaArrays(distCol, hoodCol, spdCol);

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

  private static void printConstantsUsed(
      double launchH, double targetZ, double funnelR, double funnelClear) {
    System.out.println(
        "// --- constants this run uses (direct inputs to solveFunnelClearance) ---");
    System.out.printf(
        Locale.US, "// Constants.hoodBasePose.getZ() = %.6f m (launch height)%n", launchH);
    System.out.printf(Locale.US, "// Constants.kHubTargetHeightMeters = %.6f m%n", targetZ);
    System.out.printf(Locale.US, "// Constants.kHubFunnelRadiusMeters = %.6f m%n", funnelR);
    System.out.printf(
        Locale.US,
        "// Constants.kHubFunnelHeightMeters + kHubFunnelClearanceMeters = %.6f + %.6f = %.6f m%n",
        Constants.kHubFunnelHeightMeters,
        Constants.kHubFunnelClearanceMeters,
        funnelClear);
    System.out.println("// --- KinematicsHelper.solveFunnelClearance (internal) ---");
    System.out.printf(
        Locale.US,
        "// HoodConstants.kMinAngle = %.6f rad  kMaxAngle = %.6f rad%n",
        HoodConstants.kMinAngle.in(Radians),
        HoodConstants.kMaxAngle.in(Radians));
    System.out.println("// gravity g = 9.81 m/s^2 (KinematicsHelper private constant)");
    System.out.println(
        "// initial hood/speed guess: ShotWarmstartTable.getInitialGuess (existing table in repo)");
    System.out.println("// --- this tool only (pose check comment) ---");
    System.out.printf(
        Locale.US, "// GenerateShotWarmstartTable.kLaunchOffsetM = %.2f m%n", kLaunchOffsetM);
    System.out.println("// --- optional --pose-* / motion correction ---");
    System.out.println(
        "// KinematicsHelper.getTurretPivotTranslation uses Constants.turretBasePose");
    System.out.printf(
        Locale.US,
        "// Constants.kHubPoseBlue = (%.6f, %.6f, %.6f rad)%n",
        Constants.kHubPoseBlue.getX(),
        Constants.kHubPoseBlue.getY(),
        Constants.kHubPoseBlue.getRotation().getRadians());
    System.out.println();
  }

  private static void printJavaArrays(
      List<Double> distCol, List<Double> hoodCol, List<Double> spdCol) {
    System.out.println("// Generated by GenerateShotWarmstartTable");
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
