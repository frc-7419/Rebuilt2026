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
          double targetAngleRad = targetAngleRobot.getRadians();

          // If angle goes over 180° or under -180°, flip to the other side
          double bestAngle = targetAngleRad;
          /*    if (targetAngleRad > Math.PI) {
            bestAngle = targetAngleRad - 2.0 * Math.PI;
          } else if (targetAngleRad < -Math.PI) {
            bestAngle = targetAngleRad + 2.0 * Math.PI;
          }

          Logger.recordOutput("TurretCommands/PointAtHub/TargetAngleRad", targetAngleRad);
          Logger.recordOutput("TurretCommands/PointAtHub/WrappedAngle", bestAngle);

          if (bestAngle < TurretConstants.kMinAngle.in(Radians)) {
            bestAngle = TurretConstants.kMinAngle.in(Radians);
          } else if (bestAngle > TurretConstants.kMaxAngle.in(Radians)) {
            bestAngle = TurretConstants.kMaxAngle.in(Radians);
          }*/

          Logger.recordOutput("TurretCommands/PointAtHub/ChosenAngleRad", bestAngle);

          turret.setAngle(Radians.of(bestAngle));
        },
        turret);
  }

  private static Translation2d computeFakeTargetPointLead(
      Translation2d turretPivotField, Translation2d realTargetField, Translation2d robotVelField) {

    Translation2d fakeTarget = realTargetField;

    for (int i = 0; i < 8; i++) {
      double distanceMeters = fakeTarget.getDistance(turretPivotField);
      double tofSec = timeOfFlightSeconds(distanceMeters);

      double leadScale =
          0.3; // how much to scale how much pose shifts(bascailly just shifting the scale)
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
