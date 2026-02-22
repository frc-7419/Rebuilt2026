package frc.robot.util;

import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.Constants;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.hood.HoodConstants;
import frc.robot.subsystems.shooter.ShooterConstants;
import frc.robot.subsystems.turret.TurretConstants;
import org.littletonrobotics.junction.Logger;

public final class KinematicsHelper {
  private static final double kGravity = 9.81;
  private static final int kSweepSamples = 80;
  private static final int kBisectionIters = 20;

  private static final Transform3d kRobotToTurretTransform =
      new Transform3d(
          Constants.turretBasePose.getTranslation(), Constants.turretBasePose.getRotation());

  private KinematicsHelper() {}

  public static Translation2d getTurretPivotTranslation(Pose2d robotPose) {
    return new Pose3d(robotPose).transformBy(kRobotToTurretTransform).toPose2d().getTranslation();
  }

  /** Effective aim target: target shifted by -(pivotVel * tof) for motion compensation. */
  public static Translation3d predictTargetPos(
      Translation3d target, double pivotVx, double pivotVy, double tofSec) {
    return new Translation3d(
        target.getX() - pivotVx * tofSec, target.getY() - pivotVy * tofSec, target.getZ());
  }

  /** Desired turret angle in [-π, π] nowrap. */
  public static double getDesiredTurretAngleRadHalfTurn(
      Pose2d robotPose, Translation2d turretPivotField, Translation2d aimPoint) {
    Rotation2d fieldAngle = aimPoint.minus(turretPivotField).getAngle();
    Rotation2d robotRelAngle = fieldAngle.minus(robotPose.getRotation());
    return MathUtil.angleModulus(robotRelAngle.getRadians());
  }

  /** Full-range azimuth with wrap */
  public static double calculateAzimuthAngleRad(
      Pose2d robotPose,
      Translation2d turretPivotField,
      Translation2d aimPoint,
      double currentAngleRad) {
    double desired = getDesiredTurretAngleRadHalfTurn(robotPose, turretPivotField, aimPoint);
    double twoPi = 2.0 * Math.PI;
    desired += twoPi * Math.round((currentAngleRad - desired) / twoPi);
    return MathUtil.clamp(
        desired,
        TurretConstants.kAbsoluteMinAngle.in(Radians),
        TurretConstants.kAbsoluteMaxAngle.in(Radians));
  }

  public static record FunnelShotSolution(
      double hoodAngleRad,
      double launchSpeedMps,
      double timeOfFlightSec,
      boolean funnelConstraintMet,
      double effectiveDistM,
      double effectiveFunnelClearHeightM) {}

  private static final int kFunnelRetryAttempts = 5;
  private static final double kFunnelRetryIncrementM = 0.08;
  private static final double[] kDistOffsetM = {0.0, 0.0254, 0.0508, 0.1016, 0.1524};
  private static final double[] kHubZOffsetM = {0.1016, 0.0508, 0.0};

  /**
   * Two-constraint parabola: ball must pass through funnel clearance then hub. Retries with raised
   * funnel clearance and/or hub pose up to 6 in farther until valid. Fallback is steep (max hood).
   */
  public static FunnelShotSolution solveFunnelClearance(
      double distM,
      double launchHeightM,
      double targetHeightM,
      double funnelRadiusM,
      double funnelClearHeightM,
      double fallbackLaunchSpeedMps) {
    double r = funnelRadiusM;
    double g = kGravity;
    double minHood = HoodConstants.kMinAngle.in(Radians);
    double maxHood = HoodConstants.kMaxAngle.in(Radians);

    double theta = maxHood;
    double v0 = fallbackLaunchSpeedMps;
    double tof = 0.01;
    boolean funnelMet = false;
    double usedDistM = Math.max(distM, 0.01);
    double usedFunnelClear = funnelClearHeightM;

    if (r <= 1e-6) {
      theta = maxHood;
      v0 = singleConstraintSpeed(usedDistM, targetHeightM - launchHeightM, theta, g);
      tof = usedDistM / (v0 * Math.cos(theta));
    } else {
      for (double distOffsetM : kDistOffsetM) {
        double x = Math.max(distM + distOffsetM, 0.01);
        double yDist = targetHeightM - launchHeightM;
        double A1 = x * x, B1 = x, D1 = yDist;
        double A2 = (x - r) * (x - r), B2 = (x - r);
        double Bm = -B2 / B1;
        double A3 = Bm * A1 + A2;
        if (Math.abs(A3) < 1e-9) continue;

        for (int attempt = 0; attempt < kFunnelRetryAttempts; attempt++) {
          double tryClearM = funnelClearHeightM + attempt * kFunnelRetryIncrementM;
          double hClear = tryClearM - launchHeightM;
          double D2 = hClear;
          double D3 = Bm * D1 + D2;

          double a = D3 / A3;
          double b = (D1 - A1 * a) / B1;

          if (a >= -1e-9) continue;
          double tryTheta = Math.atan(b);
          double cosTheta = Math.cos(tryTheta);
          double v0Sq = -g / (2.0 * a * cosTheta * cosTheta);
          if (v0Sq <= 0.0 || Double.isNaN(v0Sq)) continue;
          if (tryTheta < minHood || tryTheta > maxHood) continue;

          theta = tryTheta;
          v0 = Math.sqrt(v0Sq);
          tof = x / (v0 * Math.cos(theta));
          funnelMet = true;
          usedDistM = x;
          usedFunnelClear = tryClearM;
          break;
        }
        if (funnelMet) break;
      }
      if (!funnelMet) {
        for (double distOffsetM : kDistOffsetM) {
          double x = Math.max(distM + distOffsetM, 0.01);
          double yDist = targetHeightM - launchHeightM;
          double d1 = x - r;
          if (d1 <= 1e-6) continue;
          for (int attempt = 0; attempt < kFunnelRetryAttempts; attempt++) {
            double tryClearM = funnelClearHeightM + attempt * kFunnelRetryIncrementM;
            double hClear = tryClearM - launchHeightM;
            theta = maxHood;
            v0 = singleConstraintSpeed(d1, hClear, theta, g);
            if (v0 <= 0.0 || Double.isNaN(v0)) continue;
            double yAtHub = heightAtDist(x, theta, v0, g);
            if (Math.abs(yAtHub - yDist) > 0.03) continue;
            tof = x / (v0 * Math.cos(theta));
            funnelMet = true;
            usedDistM = x;
            usedFunnelClear = tryClearM;
            break;
          }
          if (funnelMet) break;
        }
        if (!funnelMet) {
          double x = Math.max(distM, 0.01);
          theta = maxHood;
          for (double zOff : kHubZOffsetM) {
            double tryTargetZ = targetHeightM + zOff;
            double yDist = tryTargetZ - launchHeightM;
            v0 = singleConstraintSpeed(x, yDist, theta, g);
            if (v0 > 0.0 && !Double.isNaN(v0) && Double.isFinite(v0)) {
              tof = x / (v0 * Math.cos(theta));
              usedDistM = x;
              break;
            }
          }
          if (tof < 1e-6) {
            double yDist = targetHeightM - launchHeightM;
            v0 = singleConstraintSpeed(x, yDist, theta, g);
            tof = x / (v0 * Math.cos(theta));
            usedDistM = x;
          }
        }
      }
    }

    Logger.recordOutput("KinematicsHelper/FunnelSolve/FunnelConstraintMet", funnelMet);
    Logger.recordOutput("KinematicsHelper/FunnelSolve/HoodAngleDeg", Math.toDegrees(theta));
    Logger.recordOutput("KinematicsHelper/FunnelSolve/LaunchSpeedMps", v0);
    Logger.recordOutput("KinematicsHelper/FunnelSolve/TOF", tof);
    Logger.recordOutput("KinematicsHelper/FunnelSolve/DistM", usedDistM);
    Logger.recordOutput(
        "KinematicsHelper/FunnelSolve/FunnelClearDeltaM", usedFunnelClear - launchHeightM);

    return new FunnelShotSolution(
        theta, v0, Math.max(tof, 0.01), funnelMet, usedDistM, usedFunnelClear);
  }

  /** Vacuum parabola height at horizontal distance d. */
  private static double heightAtDist(double d, double theta, double v0, double g) {
    double cosTheta = Math.cos(theta);
    return Math.tan(theta) * d - (g * d * d) / (2.0 * v0 * v0 * cosTheta * cosTheta);
  }

  /** Min launch speed to hit (x, yDist) at angle theta (vacuum). */
  private static double singleConstraintSpeed(double x, double yDist, double theta, double g) {
    double cosTheta = Math.cos(theta);
    double num = g * x * x;
    double den = 2.0 * cosTheta * cosTheta * (Math.tan(theta) * x - yDist);
    if (den <= 1e-9) return 8.0;
    return Math.sqrt(num / den);
  }

  public static double calculateHoodAngleRadians(
      double horizontalDistMeters,
      double launchHeightMeters,
      double targetHeightMeters,
      double launchSpeedMps) {
    double minRad = HoodConstants.kMinAngle.in(Radians);
    double maxRad = HoodConstants.kMaxAngle.in(Radians);
    ShotSolution solved =
        solveForTarget(
            Math.max(horizontalDistMeters, 0.01),
            launchHeightMeters,
            targetHeightMeters,
            Math.max(launchSpeedMps, 0.5),
            minRad,
            maxRad);
    Logger.recordOutput("KinematicsHelper/Status", solved.status);
    Logger.recordOutput("KinematicsHelper/WantedHoodAngleDeg", Math.toDegrees(solved.hoodAngleRad));
    Logger.recordOutput("KinematicsHelper/SolvedTOF", solved.timeOfFlightSec);
    return solved.hoodAngleRad;
  }

  public static double timeOfFlightSeconds(
      double horizontalDistMeters, double launchSpeedMps, double elevationAngleRad) {
    var sim =
        simulateToRange(
            Math.max(horizontalDistMeters, 0.01),
            Math.max(launchSpeedMps, 0.5),
            elevationAngleRad,
            0.0);
    if (!sim.valid) {
      double vHoriz = launchSpeedMps * Math.cos(elevationAngleRad);
      if (Math.abs(vHoriz) < 1e-6) return 1.0;
      return Math.max(horizontalDistMeters / vHoriz, 0.01);
    }
    return sim.timeSec;
  }

  public static void setHoodAngleForShot(
      Hood hood,
      double horizontalDistMeters,
      double launchHeightMeters,
      double targetHeightMeters,
      double shooterRPM) {
    double v = (shooterRPM / 60.0) * ShooterConstants.kFuelLaunchVelMetersPerSecPerRotPerSec;
    double rad =
        calculateHoodAngleRadians(horizontalDistMeters, launchHeightMeters, targetHeightMeters, v);
    hood.setAngle(Radians.of(rad));
  }

  /** Hood angle and TOF for given distance and launch speed. */
  public static ShotSolution solveForTarget(
      double horizontalDistMeters,
      double launchHeightMeters,
      double targetHeightMeters,
      double launchSpeedMps,
      double minAngleRad,
      double maxAngleRad) {
    double bestAngle = minAngleRad;
    double bestError = Double.POSITIVE_INFINITY;

    double lowRoot = Double.NaN;
    double highRoot = Double.NaN;
    double lowTof = 0.0;
    double highTof = 0.0;
    int rootsFound = 0;

    double prevAngle = minAngleRad;
    var prev = simulateToRange(horizontalDistMeters, launchSpeedMps, prevAngle, launchHeightMeters);
    double prevF = prev.zAtRange - targetHeightMeters;
    if (prev.valid) {
      bestError = Math.abs(prevF);
      bestAngle = prevAngle;
    }

    for (int i = 1; i <= kSweepSamples; i++) {
      double t = (double) i / kSweepSamples;
      double angle = MathUtil.interpolate(minAngleRad, maxAngleRad, t);
      var cur = simulateToRange(horizontalDistMeters, launchSpeedMps, angle, launchHeightMeters);
      if (!cur.valid) continue;

      double f = cur.zAtRange - targetHeightMeters;
      if (Math.abs(f) < bestError) {
        bestError = Math.abs(f);
        bestAngle = angle;
      }

      if (prev.valid && Math.signum(prevF) != Math.signum(f)) {
        var root =
            bisectRoot(
                horizontalDistMeters,
                launchHeightMeters,
                targetHeightMeters,
                launchSpeedMps,
                prevAngle,
                angle);
        if (Double.isNaN(lowRoot)) {
          lowRoot = root.hoodAngleRad;
          lowTof = root.timeOfFlightSec;
        }
        highRoot = root.hoodAngleRad;
        highTof = root.timeOfFlightSec;
        rootsFound++;
      }

      prevAngle = angle;
      prev = cur;
      prevF = f;
    }

    ShotSolution result;
    if (!Double.isNaN(highRoot)) {
      result = new ShotSolution(highRoot, highTof, "SolvedHighArc");
    } else if (!Double.isNaN(lowRoot)) {
      result = new ShotSolution(lowRoot, lowTof, "SolvedLowArc");
    } else {
      double clamped = MathUtil.clamp(bestAngle, minAngleRad, maxAngleRad);
      var est = simulateToRange(horizontalDistMeters, launchSpeedMps, clamped, launchHeightMeters);
      double tof = est.valid ? est.timeSec : 1.0;
      result = new ShotSolution(clamped, tof, "ClosestInRange");
    }

    Logger.recordOutput("KinematicsHelper/DistM", horizontalDistMeters);
    Logger.recordOutput("KinematicsHelper/LaunchHeightM", launchHeightMeters);
    Logger.recordOutput("KinematicsHelper/TargetHeightM", targetHeightMeters);
    Logger.recordOutput("KinematicsHelper/DeltaHeightM", targetHeightMeters - launchHeightMeters);
    Logger.recordOutput("KinematicsHelper/LaunchSpeedMps", launchSpeedMps);
    Logger.recordOutput("KinematicsHelper/MinAngleDeg", Math.toDegrees(minAngleRad));
    Logger.recordOutput("KinematicsHelper/MaxAngleDeg", Math.toDegrees(maxAngleRad));
    Logger.recordOutput("KinematicsHelper/RootsFound", rootsFound);
    Logger.recordOutput("KinematicsHelper/BestErrorM", bestError);
    Logger.recordOutput(
        "KinematicsHelper/LowRootDeg", Double.isNaN(lowRoot) ? -999.0 : Math.toDegrees(lowRoot));
    Logger.recordOutput(
        "KinematicsHelper/HighRootDeg", Double.isNaN(highRoot) ? -999.0 : Math.toDegrees(highRoot));
    Logger.recordOutput("KinematicsHelper/SolverStatus", result.status());
    Logger.recordOutput("KinematicsHelper/SolvedAngleDeg", Math.toDegrees(result.hoodAngleRad()));
    Logger.recordOutput("KinematicsHelper/SolvedTOF", result.timeOfFlightSec());
    var verify =
        simulateToRange(
            horizontalDistMeters, launchSpeedMps, result.hoodAngleRad(), launchHeightMeters);
    Logger.recordOutput("KinematicsHelper/VerifyZAtRangeM", verify.zAtRange);
    Logger.recordOutput("KinematicsHelper/VerifyZErrorM", verify.zAtRange - targetHeightMeters);
    Logger.recordOutput("KinematicsHelper/VerifyValid", verify.valid);

    return result;
  }

  private static ShotSolution bisectRoot(
      double horizontalDistMeters,
      double launchHeightMeters,
      double targetHeightMeters,
      double launchSpeedMps,
      double a0,
      double a1) {
    double lo = Math.min(a0, a1);
    double hi = Math.max(a0, a1);
    SimResult loRes = simulateToRange(horizontalDistMeters, launchSpeedMps, lo, launchHeightMeters);
    double loF = loRes.zAtRange - targetHeightMeters;
    SimResult bestRes = loRes;
    double bestAngle = lo;

    for (int i = 0; i < kBisectionIters; i++) {
      double mid = 0.5 * (lo + hi);
      SimResult midRes =
          simulateToRange(horizontalDistMeters, launchSpeedMps, mid, launchHeightMeters);
      if (!midRes.valid) break;
      double midF = midRes.zAtRange - targetHeightMeters;
      bestRes = midRes;
      bestAngle = mid;
      if (Math.abs(midF) < 1e-3) break;
      if (Math.signum(midF) == Math.signum(loF)) {
        lo = mid;
        loF = midF;
      } else {
        hi = mid;
      }
    }

    return new ShotSolution(bestAngle, bestRes.timeSec, "Bisection");
  }

  /** Trajectory simulation to find z at given horizontal range. */
  private static SimResult simulateToRange(
      double targetX, double launchSpeed, double elevationRad, double launchHeight) {
    double vx = launchSpeed * Math.cos(elevationRad);
    double vz = launchSpeed * Math.sin(elevationRad);
    if (vx <= 1e-9) return new SimResult(false, launchHeight, 0.0);

    double tToTarget = targetX / vx;
    double tGround = (vz + Math.sqrt(vz * vz + 2.0 * kGravity * launchHeight)) / kGravity;
    if (tToTarget > tGround) return new SimResult(false, 0.0, tToTarget);

    double zAt = launchHeight + vz * tToTarget - 0.5 * kGravity * tToTarget * tToTarget;
    return new SimResult(true, zAt, tToTarget);
  }

  public static record ShotSolution(double hoodAngleRad, double timeOfFlightSec, String status) {}

  private static record SimResult(boolean valid, double zAtRange, double timeSec) {}
}
