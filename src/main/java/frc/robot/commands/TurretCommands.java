// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotState;
import frc.robot.subsystems.turret.Turret;
import frc.robot.subsystems.turret.TurretConstants;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public final class TurretCommands {
  private TurretCommands() {}

  /** Manual joystick turret control. Expects input in [-1, 1]. */
  public static Command joystickTurret(Turret turret, DoubleSupplier input) {
    return Commands.run(
        () -> {
          double val = MathUtil.applyDeadband(input.getAsDouble(), 0.05);
          turret.setOpenLoop(val * 12.0); // scale to volts
        },
        turret);
  }

  /** Hold turret at a specific absolute rotation. */
  public static Command holdAngle(Turret turret, Angle angle) {
    return Commands.runOnce(() -> turret.setAngle(angle), turret).withTimeout(0.0);
  }

  /**
   * Points the turret towards the hub. Blue alliance target: (4.620, 4.030). Red alliance target is
   * calculated as the symmetric point on the field.
   */
  public static Command pointAtHub(Turret turret) {
    return Commands.run(
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

          Translation2d targetPoint;
          if (state.isRedAlliance()) {
            targetPoint = new Translation2d(fieldLength - hubXBlue, hubYBlue);
          } else {
            targetPoint = new Translation2d(hubXBlue, hubYBlue);
          }

          Translation2d turretPivotField =
              robotPose
                  .getTranslation()
                  .plus(
                      TurretConstants.kTurretOffset
                          .getTranslation()
                          .rotateBy(robotPose.getRotation()));

          Translation2d turretToTarget = targetPoint.minus(turretPivotField);
          Rotation2d targetAngleField = turretToTarget.getAngle();
          Rotation2d targetAngleRobot = targetAngleField.minus(robotPose.getRotation());
          double targetAngleRad = targetAngleRobot.getRadians();

          // If angle goes over 180° or under -180°, flip to the other side
          double bestAngle = targetAngleRad;
          if (targetAngleRad > Math.PI) {
            bestAngle = targetAngleRad - 2.0 * Math.PI;
          } else if (targetAngleRad < -Math.PI) {
            bestAngle = targetAngleRad + 2.0 * Math.PI;
          }

          Logger.recordOutput("TurretCommands/PointAtHub/TargetAngleRad", targetAngleRad);
          Logger.recordOutput("TurretCommands/PointAtHub/WrappedAngle", bestAngle);

          if (bestAngle < TurretConstants.kMinAngleRad) {
            bestAngle = TurretConstants.kMinAngleRad;
          } else if (bestAngle > TurretConstants.kMaxAngleRad) {
            bestAngle = TurretConstants.kMaxAngleRad;
          }

          Logger.recordOutput("TurretCommands/PointAtHub/ChosenAngleRad", bestAngle);

          turret.setAngle(Radians.of(bestAngle));
        },
        turret);
  }
}
