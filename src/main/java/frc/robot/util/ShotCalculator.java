package frc.robot.util;

import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.Constants;
import frc.robot.subsystems.hood.HoodConstants;
import frc.robot.subsystems.shooter.ShooterConstants;
import frc.robot.subsystems.turret.TurretConstants;

/**
 * One immutable solution per scheduler cycle. Inputs and clock are explicit for deterministic tests
 * and replay. The stationary funnel mathematics and speed conversion remain in their original form.
 */
public final class ShotCalculator {
  public static final double LAUNCH_OFFSET_M = 0.28;
  public static final double MAX_POSE_AGE_SEC = 0.1;
  public static final double MAX_RELEASE_DELAY_SEC = 0.5;
  private static final int MAX_ITERATIONS = 20;
  private static final double TARGET_TOLERANCE_M = 0.01;

  public record Input(
      double nowSec,
      double poseTimestampSec,
      Pose2d pose,
      double robotVx,
      double robotVy,
      double omegaRadPerSec,
      double currentTurretRad,
      Translation3d target,
      boolean hubMode,
      boolean passCenter,
      boolean fullRange,
      double launchVelConstant,
      double releaseDelaySec,
      boolean compensateMotion,
      boolean useCalibration,
      long poseResetSequence,
      double counterRotationScale) {}

  public record Solution(
      double turretAngleRad,
      double turretVelocityRadPerSec,
      double hoodAngleRad,
      double wheelRpm,
      double timeOfFlightSec,
      Translation3d virtualTarget,
      double distanceM,
      boolean converged,
      boolean valid,
      String reason,
      int iterations,
      Pose2d releasePose,
      Translation2d pivotVelocity,
      boolean measuredCalibration) {}

  private record Arc(double hood, double rpm, double tof, boolean valid, boolean measured) {}

  private final ShotCalibration calibration;
  private Solution cached;
  private Solution lastFinite;
  private Input previousInput;
  private double previousFieldBearing;
  private double previousTurretAngle;
  private final double[] bearingRates = new double[3];
  private int rateCount;
  private int rateIndex;

  public ShotCalculator() {
    this(ShotCalibration.MEASURED);
  }

  public ShotCalculator(ShotCalibration calibration) {
    this.calibration = calibration;
  }

  /** Call before the scheduler; calculate lazily after subsystem inputs and vision have updated. */
  public void beginCycle() {
    cached = null;
  }

  public void resetTracking() {
    previousInput = null;
    rateCount = 0;
    rateIndex = 0;
    cached = null;
  }

  public Solution calculate(Input in) {
    if (cached != null) return cached;
    if (!finite(
            in.nowSec(),
            in.poseTimestampSec(),
            in.pose().getX(),
            in.pose().getY(),
            in.pose().getRotation().getRadians(),
            in.robotVx(),
            in.robotVy(),
            in.omegaRadPerSec(),
            in.currentTurretRad(),
            in.target().getX(),
            in.target().getY(),
            in.target().getZ(),
            in.launchVelConstant(),
            in.releaseDelaySec(),
            in.counterRotationScale())
        || in.launchVelConstant() <= 0.0
        || in.releaseDelaySec() < 0.0
        || in.releaseDelaySec() > MAX_RELEASE_DELAY_SEC) {
      return hold(in, "INVALID_INPUT", false, 0);
    }
    double age = in.nowSec() - in.poseTimestampSec();
    if (age < -0.005 || age > MAX_POSE_AGE_SEC) return hold(in, "STALE_POSE", false, 0);

    Pose2d releasePose =
        predictReleasePose(
            in.pose(),
            in.robotVx(),
            in.robotVy(),
            in.omegaRadPerSec(),
            Math.max(0.0, age) + in.releaseDelaySec());
    Translation2d pivot = KinematicsHelper.getTurretPivotTranslation(releasePose);
    ChassisSpeeds field =
        ChassisSpeeds.fromRobotRelativeSpeeds(
            in.robotVx(), in.robotVy(), in.omegaRadPerSec(), releasePose.getRotation());
    Translation2d offset = pivot.minus(releasePose.getTranslation());
    Translation2d pivotVelocity =
        new Translation2d(
            field.vxMetersPerSecond - in.omegaRadPerSec() * offset.getY(),
            field.vyMetersPerSecond + in.omegaRadPerSec() * offset.getX());
    double staticDistance = pivot.getDistance(in.target().toTranslation2d());
    boolean stationary =
        Math.abs(field.vxMetersPerSecond) < 0.1
            && Math.abs(field.vyMetersPerSecond) < 0.1
            && Math.abs(in.omegaRadPerSec()) < 0.1;
    Translation3d aim = in.target();
    Arc arc = null;
    boolean converged = false;
    int iterations = 0;
    for (; iterations < MAX_ITERATIONS; ) {
      iterations++;
      double distance = pivot.getDistance(aim.toTranslation2d());
      arc = solveArc(in, distance, staticDistance);
      if (!finite(arc.hood(), arc.rpm(), arc.tof()) || arc.tof() <= 0) {
        return hold(in, "NONFINITE_SOLUTION", false, iterations);
      }
      Translation3d next =
          stationary || !in.compensateMotion()
              ? aim
              : KinematicsHelper.predictTargetPos(
                  in.target(), pivotVelocity.getX(), pivotVelocity.getY(), arc.tof());
      // Keep the arc and aim from the same iteration; this residual also bounds the flight-time
      // mismatch. Never advance the aim without solving the corresponding arc.
      if (next.getDistance(aim) < TARGET_TOLERANCE_M) {
        converged = true;
        break;
      }
      if (iterations < MAX_ITERATIONS) aim = next;
    }
    if (!converged) return hold(in, "NOT_CONVERGED", false, iterations);
    if (!arc.valid()) return hold(in, "INVALID_ARC", true, iterations);
    double fieldBearing = aim.toTranslation2d().minus(pivot).getAngle().getRadians();
    double raw = MathUtil.angleModulus(fieldBearing - releasePose.getRotation().getRadians());
    double min = TurretConstants.kAbsoluteMinAngle.in(Radians);
    double max = TurretConstants.kAbsoluteMaxAngle.in(Radians);
    double turret =
        in.fullRange()
            ? nearestReachableAngle(raw, in.currentTurretRad(), min, max)
            : MathUtil.clamp(raw, Math.max(min, -Math.PI), Math.min(max, Math.PI));
    if (Math.abs(MathUtil.angleModulus(turret - raw)) > 1e-6) {
      return hold(in, "TURRET_UNREACHABLE", true, iterations);
    }
    double trackingRate = 0.0;
    if (canDifferentiate(in, turret, fieldBearing)) {
      bearingRates[rateIndex] =
          MathUtil.angleModulus(fieldBearing - previousFieldBearing)
              / (in.nowSec() - previousInput.nowSec());
      rateIndex = (rateIndex + 1) % bearingRates.length;
      rateCount = Math.min(rateCount + 1, bearingRates.length);
      for (int i = 0; i < rateCount; i++) trackingRate += bearingRates[i];
      trackingRate /= rateCount;
    } else {
      rateCount = 0;
      rateIndex = 0;
    }
    previousInput = in;
    previousFieldBearing = fieldBearing;
    previousTurretAngle = turret;
    double velocity = trackingRate - in.omegaRadPerSec() * in.counterRotationScale();
    if ((turret <= min + 1e-4 && velocity < 0) || (turret >= max - 1e-4 && velocity > 0)) {
      velocity = 0.0;
    }
    double hood = inObstacleZone(pivot.getX()) ? HoodConstants.kMaxAngle.in(Radians) : arc.hood();
    cached =
        new Solution(
            turret,
            velocity,
            hood,
            arc.rpm(),
            arc.tof(),
            aim,
            pivot.getDistance(aim.toTranslation2d()),
            true,
            true,
            "OK",
            iterations,
            releasePose,
            pivotVelocity,
            arc.measured());
    lastFinite = cached;
    return cached;
  }

  private Arc solveArc(Input in, double distance, double staticDistance) {
    if (in.hubMode()) {
      double launchDistance = Math.max(0.01, distance - LAUNCH_OFFSET_M);
      var measured =
          in.useCalibration()
              ? calibration.sample(launchDistance)
              : java.util.Optional.<ShotCalibration.Point>empty();
      if (measured.isPresent()) {
        var p = measured.get();
        return new Arc(p.hoodRad(), p.wheelRpm(), p.flightTimeSec(), true, true);
      }
      double radius =
          staticDistance > 1e-6
              ? Constants.kHubFunnelRadiusMeters * distance / staticDistance
              : Constants.kHubFunnelRadiusMeters;
      var a =
          KinematicsHelper.solveFunnelClearance(
              launchDistance,
              Constants.hoodBasePose.getZ(),
              in.target().getZ(),
              radius,
              Constants.kHubFunnelHeightMeters + Constants.kHubFunnelClearanceMeters,
              ShooterConstants.kAutoAimRPM / 60.0 * in.launchVelConstant());
      return new Arc(
          a.hoodAngleRad(),
          toRpm(a.launchSpeedMps(), in.launchVelConstant()),
          a.timeOfFlightSec(),
          a.funnelConstraintMet(),
          false);
    }
    double angle =
        (in.passCenter() ? HoodConstants.kMaxAngle : HoodConstants.kMinAngle).in(Radians);
    var a =
        KinematicsHelper.solveForTargetHighArc(
            distance, Constants.hoodBasePose.getZ(), in.target().getZ(), angle);
    return new Arc(a[0], toRpm(a[1], in.launchVelConstant()), a[2], true, false);
  }

  private boolean canDifferentiate(Input in, double turret, double bearing) {
    if (previousInput == null) return false;
    double dt = in.nowSec() - previousInput.nowSec();
    return dt >= 0.005
        && dt <= MAX_POSE_AGE_SEC
        && in.poseResetSequence() == previousInput.poseResetSequence()
        && in.target().equals(previousInput.target())
        && in.hubMode() == previousInput.hubMode()
        && in.passCenter() == previousInput.passCenter()
        && in.fullRange() == previousInput.fullRange()
        && in.compensateMotion() == previousInput.compensateMotion()
        && in.useCalibration() == previousInput.useCalibration()
        && in.releaseDelaySec() == previousInput.releaseDelaySec()
        && in.launchVelConstant() == previousInput.launchVelConstant()
        && Math.abs(turret - previousTurretAngle) < Math.PI
        && Math.abs(MathUtil.angleModulus(bearing - previousFieldBearing)) < Math.PI / 2;
  }

  private Solution hold(Input in, String reason, boolean converged, int iterations) {
    resetTracking();
    Solution last = lastFinite;
    double min = TurretConstants.kAbsoluteMinAngle.in(Radians);
    double max = TurretConstants.kAbsoluteMaxAngle.in(Radians);
    double initialAngle =
        Double.isFinite(in.currentTurretRad())
            ? MathUtil.clamp(in.currentTurretRad(), min, max)
            : 0.0;
    cached =
        new Solution(
            last == null ? initialAngle : last.turretAngleRad(),
            0.0,
            last == null ? HoodConstants.kMaxAngle.in(Radians) : last.hoodAngleRad(),
            last == null ? 0.0 : last.wheelRpm(),
            last == null ? 0.0 : last.timeOfFlightSec(),
            last == null ? new Translation3d() : last.virtualTarget(),
            last == null ? 0.0 : last.distanceM(),
            converged,
            false,
            reason,
            iterations,
            last == null ? Pose2d.kZero : last.releasePose(),
            last == null ? Translation2d.kZero : last.pivotVelocity(),
            last != null && last.measuredCalibration());
    return cached;
  }

  public static Pose2d predictReleasePose(
      Pose2d pose, double vx, double vy, double omega, double dt) {
    return pose.exp(new Twist2d(vx * dt, vy * dt, omega * dt));
  }

  /** Enumerate feasible windings before choosing, rather than clamping an infeasible winding. */
  public static double nearestReachableAngle(double raw, double current, double min, double max) {
    double turn = 2 * Math.PI;
    long first = (long) Math.ceil((min - raw) / turn);
    long last = (long) Math.floor((max - raw) / turn);
    if (first > last) return MathUtil.clamp(raw, min, max);
    long winding = Math.max(first, Math.min(last, Math.round((current - raw) / turn)));
    return raw + winding * turn;
  }

  public static boolean inObstacleZone(double x) {
    return (x >= 4.472 - 0.33 && x <= 5.0 + 0.33)
        || (x >= 16.54 - 5.0 - 0.33 && x <= 16.54 - 4.472 + 0.33);
  }

  private static double toRpm(double speed, double constant) {
    return MathUtil.clamp(
        speed / constant * 60.0, ShooterConstants.kAutoAimRPMMin, ShooterConstants.kAutoAimRPMMax);
  }

  private static boolean finite(double... values) {
    for (double value : values) if (!Double.isFinite(value)) return false;
    return true;
  }
}
