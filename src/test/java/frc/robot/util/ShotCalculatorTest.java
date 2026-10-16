package frc.robot.util;

import static edu.wpi.first.units.Units.Radians;
import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.Constants;
import frc.robot.subsystems.hood.HoodConstants;
import frc.robot.subsystems.shooter.ShooterConstants;
import java.util.List;
import org.junit.jupiter.api.Test;

class ShotCalculatorTest {
  private static final double VEL = ShooterConstants.kFuelLaunchVelMetersPerSecPerRotPerSec;
  private static final Translation3d HUB =
      new Translation3d(
          Constants.kHubPoseBlue.getX(),
          Constants.kHubPoseBlue.getY(),
          Constants.kHubTargetHeightMeters);

  private ShotCalculator.Input input(
      double t,
      double sample,
      Pose2d pose,
      double vx,
      double vy,
      double omega,
      double current,
      Translation3d target,
      boolean hub,
      long reset) {
    return new ShotCalculator.Input(
        t, sample, pose, vx, vy, omega, current, target, hub, false, true, VEL, 0.0, true, false,
        reset, 1.0);
  }

  private Pose2d poseAtRange(double launchDistance) {
    return new Pose2d(
        HUB.getX()
            - launchDistance
            - ShotCalculator.LAUNCH_OFFSET_M
            - Constants.turretBasePose.getX(),
        HUB.getY() - Constants.turretBasePose.getY(),
        Rotation2d.kZero);
  }

  @Test
  void stationarySettingsMatchOriginalFunnelSolver() {
    for (double distance : new double[] {1.5, 2, 2.5, 3, 4, 5, 6}) {
      var expected =
          KinematicsHelper.solveFunnelClearance(
              distance,
              Constants.hoodBasePose.getZ(),
              HUB.getZ(),
              Constants.kHubFunnelRadiusMeters,
              Constants.kHubFunnelHeightMeters + Constants.kHubFunnelClearanceMeters,
              ShooterConstants.kAutoAimRPM / 60.0 * VEL);
      assertTrue(expected.funnelConstraintMet(), "baseline range " + distance);
      var actual =
          new ShotCalculator()
              .calculate(input(1, 1, poseAtRange(distance), 0, 0, 0, 0, HUB, true, 0));
      assertTrue(actual.valid(), actual.reason());
      assertEquals(expected.hoodAngleRad(), actual.hoodAngleRad(), 1e-10);
      assertEquals(
          MathUtil.clamp(
              expected.launchSpeedMps() / VEL * 60,
              ShooterConstants.kAutoAimRPMMin,
              ShooterConstants.kAutoAimRPMMax),
          actual.wheelRpm(),
          1e-8);
      assertEquals(expected.timeOfFlightSec(), actual.timeOfFlightSec(), 1e-10);
      assertEquals(1, actual.iterations());
    }
  }

  @Test
  void movingSolutionsConvergeToTheVelocityCompensatedTarget() {
    for (double[] motion :
        new double[][] {{0, 1, 0}, {0, -2, 0}, {1, 0, 0}, {-2, 0, 0}, {0, 0, 1}, {1, 1, -1}}) {
      var s =
          new ShotCalculator()
              .calculate(
                  input(1, 1, poseAtRange(3), motion[0], motion[1], motion[2], 0, HUB, true, 0));
      assertTrue(s.valid(), java.util.Arrays.toString(motion) + " " + s.reason());
      assertTrue(s.iterations() <= 20);
      var expected =
          KinematicsHelper.predictTargetPos(
              HUB, s.pivotVelocity().getX(), s.pivotVelocity().getY(), s.timeOfFlightSec());
      assertTrue(expected.getDistance(s.virtualTarget()) < 0.01);
      double distance =
          KinematicsHelper.getTurretPivotTranslation(s.releasePose())
              .getDistance(s.virtualTarget().toTranslation2d());
      assertEquals(distance, s.distanceM(), 1e-10);
    }
  }

  @Test
  void translationFeedsForwardTheChangingBearing() {
    for (double vy : new double[] {-1, 1}) {
      var calculator = new ShotCalculator();
      Pose2d pose = poseAtRange(3);
      calculator.calculate(input(1, 1, pose, 0, vy, 0, 0, HUB, true, 0));
      calculator.beginCycle();
      var s =
          calculator.calculate(
              input(
                  1.02,
                  1.02,
                  new Pose2d(
                      pose.getTranslation().plus(new Translation2d(0, vy * .02)),
                      pose.getRotation()),
                  0,
                  vy,
                  0,
                  0,
                  HUB,
                  true,
                  0));
      assertTrue(s.valid(), s.reason());
      assertTrue(s.turretVelocityRadPerSec() * vy < 0);
    }
  }

  @Test
  void rotationUsesPivotOffsetAndCounterRotation() {
    var s = new ShotCalculator().calculate(input(1, 1, poseAtRange(3), 0, 0, 1, 0, HUB, true, 0));
    assertTrue(s.valid(), s.reason());
    assertEquals(-Constants.turretBasePose.getY(), s.pivotVelocity().getX(), 1e-10);
    assertEquals(Constants.turretBasePose.getX(), s.pivotVelocity().getY(), 1e-10);
    assertEquals(-1, s.turretVelocityRadPerSec(), 1e-10);
  }

  @Test
  void predictsCurvedTranslationAndHeadingThroughSensorAgeAndReleaseDelay() {
    Pose2d pose = poseAtRange(3);
    var base = input(1.04, 1, pose, 1, .5, 1, 0, HUB, true, 0);
    var withDelay =
        new ShotCalculator.Input(
            base.nowSec(),
            base.poseTimestampSec(),
            pose,
            1,
            .5,
            1,
            0,
            HUB,
            true,
            false,
            true,
            VEL,
            .03,
            true,
            false,
            0,
            1);
    var s = new ShotCalculator().calculate(withDelay);
    assertTrue(s.valid(), s.reason());
    assertEquals(ShotCalculator.predictReleasePose(pose, 1, .5, 1, .07), s.releasePose());
    assertEquals(.07, s.releasePose().getRotation().getRadians(), 1e-10);
  }

  @Test
  void cachedResultIsSharedUntilTheNextCycle() {
    var c = new ShotCalculator();
    var a = c.calculate(input(1, 1, poseAtRange(3), 0, 0, 0, 0, HUB, true, 0));
    var b = c.calculate(input(2, 1, poseAtRange(3), 2, 1, 0, 0, HUB, true, 0));
    assertSame(a, b);
    c.beginCycle();
    assertFalse(c.calculate(input(2, 1, poseAtRange(3), 2, 1, 0, 0, HUB, true, 0)).valid());
  }

  @Test
  void staleAndNonfiniteInputsHoldTheLastFiniteTargetsWithNoFeedforward() {
    var c = new ShotCalculator();
    var good = c.calculate(input(1, 1, poseAtRange(3), 0, 1, 0, 0, HUB, true, 0));
    for (double vx : new double[] {0, Double.NaN}) {
      c.beginCycle();
      var bad = c.calculate(input(2, 1, poseAtRange(3), vx, 1, 1, 0, HUB, true, 0));
      assertFalse(bad.valid());
      assertEquals(good.turretAngleRad(), bad.turretAngleRad());
      assertEquals(good.hoodAngleRad(), bad.hoodAngleRad());
      assertEquals(good.wheelRpm(), bad.wheelRpm());
      assertEquals(0, bad.turretVelocityRadPerSec());
    }
  }

  @Test
  void targetChangesPoseResetsAndEnableResetsDiscardDerivativeHistory() {
    var c = new ShotCalculator();
    var pose = poseAtRange(3);
    c.calculate(input(1, 1, pose, 0, 1, 0, 0, HUB, true, 0));
    c.beginCycle();
    var changedTarget = HUB.plus(new Translation3d(0, 1, 0));
    assertEquals(
        0,
        c.calculate(input(1.02, 1.02, pose, 0, 1, 0, 0, changedTarget, true, 0))
            .turretVelocityRadPerSec());
    c.beginCycle();
    assertEquals(
        0,
        c.calculate(input(1.04, 1.04, pose, 0, 1, 0, 0, changedTarget, true, 1))
            .turretVelocityRadPerSec());
    c.resetTracking();
    assertEquals(
        0,
        c.calculate(input(1.06, 1.06, pose, 0, 1, 0, 0, changedTarget, true, 1))
            .turretVelocityRadPerSec());
  }

  @Test
  void nearestReachableWindingDoesNotClampAnUnreachableEquivalent() {
    assertEquals(
        Math.toRadians(-100),
        ShotCalculator.nearestReachableAngle(
            Math.toRadians(-100), Math.toRadians(240), Math.toRadians(-250), Math.toRadians(250)),
        1e-10);
    assertEquals(
        Math.toRadians(181),
        ShotCalculator.nearestReachableAngle(
            Math.toRadians(-179), Math.toRadians(179), Math.toRadians(-250), Math.toRadians(250)),
        1e-10);
  }

  @Test
  void outwardVelocityIsSuppressedAtThePhysicalLimit() {
    Pose2d pose = new Pose2d(0, 0, Rotation2d.kZero);
    Translation2d pivot = KinematicsHelper.getTurretPivotTranslation(pose);
    Translation2d xy = pivot.plus(new Translation2d(4, Rotation2d.fromDegrees(-110)));
    var s =
        new ShotCalculator()
            .calculate(
                new ShotCalculator.Input(
                    1,
                    1,
                    pose,
                    0,
                    0,
                    -1,
                    Math.toRadians(250),
                    new Translation3d(xy.getX(), xy.getY(), 0),
                    false,
                    false,
                    true,
                    VEL,
                    0,
                    false,
                    false,
                    0,
                    1));
    assertTrue(s.valid(), s.reason());
    assertEquals(Math.toRadians(250), s.turretAngleRad(), 1e-10);
    assertEquals(0, s.turretVelocityRadPerSec());
  }

  @Test
  void wrappingFieldBearingDoesNotProduceAFullTurnVelocitySpike() {
    var c = new ShotCalculator();
    var target = new Translation3d(-4, 0, 0);
    Pose2d a =
        new Pose2d(
            -Constants.turretBasePose.getX(),
            -.01 - Constants.turretBasePose.getY(),
            Rotation2d.kZero);
    var first = c.calculate(input(1, 1, a, 0, 0, 0, Math.PI, target, false, 0));
    c.beginCycle();
    Pose2d b = new Pose2d(a.getX(), a.getY() + .02, Rotation2d.kZero);
    var second =
        c.calculate(input(1.02, 1.02, b, 0, 0, 0, first.turretAngleRad(), target, false, 0));
    assertTrue(second.valid());
    assertTrue(Math.abs(second.turretAngleRad() - first.turretAngleRad()) < .02);
    assertTrue(Math.abs(second.turretVelocityRadPerSec()) < 1);
  }

  @Test
  void boundedIterationReportsNonconvergenceInsteadOfEmittingWildTargets() {
    var s =
        new ShotCalculator().calculate(input(1, 1, poseAtRange(3), -100, 0, 0, 0, HUB, true, 0));
    assertFalse(s.valid());
    assertEquals("NOT_CONVERGED", s.reason());
    assertEquals(20, s.iterations());
    assertEquals(0, s.turretVelocityRadPerSec());
    assertTrue(Double.isFinite(s.wheelRpm()));
  }

  @Test
  void measuredTablesAreCompleteOptInAndBoundedToTheirCalibratedRange() {
    double hood = HoodConstants.kMaxAngle.in(Radians);
    var table =
        new ShotCalibration(
            List.of(
                new ShotCalibration.Point(2, hood, 2400, .8),
                new ShotCalibration.Point(4, hood, 3000, 1.0)));
    var c = new ShotCalculator(table);
    var original = input(1, 1, poseAtRange(3), 0, 0, 0, 0, HUB, true, 0);
    assertFalse(c.calculate(original).measuredCalibration());
    c.beginCycle();
    var s =
        c.calculate(
            new ShotCalculator.Input(
                1,
                1,
                poseAtRange(3),
                0,
                0,
                0,
                0,
                HUB,
                true,
                false,
                true,
                VEL,
                0,
                true,
                true,
                0,
                1));
    assertTrue(s.measuredCalibration());
    assertEquals(2700, s.wheelRpm(), 1e-8);
    assertEquals(.9, s.timeOfFlightSec(), 1e-10);
    assertTrue(table.sample(1).isEmpty());
    assertTrue(table.sample(5).isEmpty());
    assertThrows(
        IllegalArgumentException.class,
        () -> new ShotCalibration(List.of(new ShotCalibration.Point(2, hood, 2400, Double.NaN))));
    c.beginCycle();
    assertFalse(
        c.calculate(
                new ShotCalculator.Input(
                    1,
                    1,
                    poseAtRange(3),
                    0,
                    0,
                    0,
                    0,
                    new Translation3d(HUB.getX(), HUB.getY(), 0),
                    false,
                    false,
                    true,
                    VEL,
                    0,
                    true,
                    true,
                    0,
                    1))
            .measuredCalibration());
  }
}
