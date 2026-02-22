package frc.robot.commands;

import static edu.wpi.first.math.MathUtil.*;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.wpilibj2.command.Commands.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotState;
import frc.robot.subsystems.turret.Turret;
import frc.robot.subsystems.turret.TurretConstants;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public final class TurretCommands {
  private static Translation2d filteredVelField = new Translation2d();
  private static double lastTime = Timer.getFPGATimestamp();

  private TurretCommands() {}

  /** Manual joystick turret control. Expects input in [-1, 1]. */
  public static Command joystickTurret(Turret turret, DoubleSupplier input) {
    return run(
        () -> {
          double val = applyDeadband(input.getAsDouble(), 0.05);
          turret.setOpenLoop(val * 12.0); // scale to volts
        },
        turret);
  }

  /** Hold turret at a specific absolute rotation. */
  public static Command holdAngle(Turret turret, Angle angle) {
    return runOnce(() -> turret.setAngle(angle), turret).withTimeout(0.0);
  }

  public static Command toTurretPosition(Turret turret, Angle position) {

    return runOnce(() -> turret.setAngle(position), turret);
  }

  /**
   * Points the turret towards the hub. Blue alliance target: (4.620, 4.030). Red alliance target is
   * calculated as the symmetric point on the field.
   */
  public static Command pointAtHub(Turret turret) {
    return run(
        () -> {
          RobotState state = RobotState.getInstance();
          var latestRobotPose = state.getLatestFieldToRobot();

          if (latestRobotPose == null) {
            return;
          }

          var robotPose = latestRobotPose.getValue();

          double hubXBlue = 4.620;
          double hubYBlue = 4.030;
          double fieldLength = 16.54;

          Translation2d realTargetPoint;
          if (state.isRedAlliance()) {
            realTargetPoint = new Translation2d(fieldLength - hubXBlue, hubYBlue);
          } else {
            realTargetPoint = new Translation2d(hubXBlue, hubYBlue);
          }

          Translation2d turretPivotField =
              robotPose
                  .getTranslation()
                  .plus(
                      TurretConstants.kTurretOffset
                          .getTranslation()
                          .rotateBy(robotPose.getRotation()));

          var robotRel = state.getLatestRobotRelativeChassisSpeed();

          // robot-relative velocity
          Translation2d robotVelRobot =
              new Translation2d(robotRel.vxMetersPerSecond, robotRel.vyMetersPerSecond);

          // convert to field-relative using robot yaw
          Rotation2d robotYawField = robotPose.getRotation();
          Translation2d robotVelField = robotVelRobot.rotateBy(robotYawField);

          double now = Timer.getFPGATimestamp();
          double dt = now - lastTime;
          lastTime = now;

          double tau = 0.15; // bascailly ups the damping to make it not go crazy
          double alpha = (dt <= 0.0) ? 1.0 : dt / (tau + dt);

          filteredVelField =
              filteredVelField.plus(robotVelField.minus(filteredVelField).times(alpha));
          robotVelField = filteredVelField;

          Translation2d fakeTargetPoint =
              computeFakeTargetPointLead(turretPivotField, realTargetPoint, robotVelField);

          Logger.recordOutput(
              "Turret/FakeTargetPose", new Pose2d(fakeTargetPoint, new Rotation2d()));

          Logger.recordOutput("TurretCommands/PointAtHub/RealTarget", realTargetPoint);
          Logger.recordOutput("TurretCommands/PointAtHub/FakeTarget", fakeTargetPoint);
          Logger.recordOutput("TurretCommands/PointAtHub/RobotVelField", robotVelField);

          Translation2d turretToTarget = fakeTargetPoint.minus(turretPivotField);
          Rotation2d targetAngleField = turretToTarget.getAngle();
          Rotation2d targetAngleRobot = targetAngleField.minus(robotPose.getRotation());
          double targetAngleRad = angleModulus(targetAngleRobot.getRadians());

          double currentPosRad = turret.getAngle().in(Radians);

          double kBoundaryTolerance = state.isShooting() ? Math.toRadians(40) : Math.toRadians(15);
          double twoPi = 2.0 * Math.PI;

          double shortPathTarget =
              targetAngleRad + twoPi * Math.round((currentPosRad - targetAngleRad) / twoPi);

          double bestAngle;

          if (Math.abs(targetAngleRad) > Math.PI - kBoundaryTolerance) {
            bestAngle =
                clamp(
                    shortPathTarget,
                    TurretConstants.kAbsoluteMinAngle.in(Radians),
                    TurretConstants.kAbsoluteMaxAngle.in(Radians));
          } else {
            bestAngle = angleModulus(targetAngleRad);
          }

          Logger.recordOutput("TurretCommands/PointAtHub/TargetAngleRad", targetAngleRad);
          Logger.recordOutput("TurretCommands/PointAtHub/ChosenAngleRad", bestAngle);

          turret.setAngle(Radians.of(bestAngle));
        },
        turret);
  }

  /**
   * Computes a lead target point so the turret aims where the hub will be relative to us when the
   * shot arrives. Uses iterative convergence: fakeTarget = realTarget - robotVel * tof * leadScale,
   * where tof is time-of-flight from turret to current fake target. Correct because we aim "back"
   * along our velocity so that after we move for tof seconds the shot lines up.
   */
  private static Translation2d computeFakeTargetPointLead(
      Translation2d turretPivotField, Translation2d realTargetField, Translation2d robotVelField) {

    Translation2d fakeTarget = realTargetField;
    double leadScale = 0.3; // scale for how much to shift aim (tuning)

    for (int i = 0; i < 8; i++) {
      double distanceMeters = fakeTarget.getDistance(turretPivotField);
      double tofSec = timeOfFlightSeconds(distanceMeters);
      Translation2d drift = robotVelField.times(tofSec * leadScale);
      fakeTarget = realTargetField.minus(drift);
    }

    return fakeTarget;
  }

  private static double timeOfFlightSeconds(double distanceMeters) {
    double distanceInches = edu.wpi.first.math.util.Units.metersToInches(distanceMeters);

    if (distanceInches < 113.0) {
      return ((0.03 / 30.0) * (distanceInches - 113.0)) + (23.0 / 30.0);
    } else {
      return ((0.041 / 30.0) * (distanceInches - 113.0)) + (23.0 / 30.0);
    }
  }
}
