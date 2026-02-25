package frc.robot.simulation;

import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.Constants;
import frc.robot.RobotState;
import frc.robot.commands.AutoAim;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.turret.Turret;

/**
 * Helper to launch fuel from the shooter in simulation. Spawns at turret-centered launch point
 * (with offset so ball clears robot) with correct velocity so FuelSim ball leaves from the turret.
 */
public final class FuelSimLaunch {

  /**
   * Forward offset from turret center to launch point (m). Puts the ball outside the robot so
   * FuelSim doesn't treat it as an immediate collision. No physical barrel; flywheel at output.
   */
  private static final double kLaunchOffsetMeters = 0.28;

  private FuelSimLaunch() {}

  public static void launchFromShooter(
      FuelSim fuelSim, Drive drive, Shooter shooter, Hood hood, Turret turret) {
    if (fuelSim == null) return;
    double rpm = shooter.getRPM();
    double rotPerSec = rpm / 60.0;
    double launchMps = rotPerSec * AutoAim.getLaunchVelConstant();
    double hoodRad = hood.getAngle().in(Radians);
    double turretRad = turret.getAngle().in(Radians);
    Pose2d robot = drive.getPose();

    // Center launch on turret axis (x,y); height from hood pivot z
    double tx = Constants.turretBasePose.getX();
    double ty = Constants.turretBasePose.getY();
    double hz = Constants.hoodBasePose.getZ();
    double ch = Math.cos(hoodRad), sh = Math.sin(hoodRad);
    double ct = Math.cos(turretRad), st = Math.sin(turretRad);
    double rx = tx + kLaunchOffsetMeters * ch * ct;
    double ry = ty + kLaunchOffsetMeters * ch * st;
    double rz = hz + kLaunchOffsetMeters * sh;

    double rCos = robot.getRotation().getCos(), rSin = robot.getRotation().getSin();
    double releaseX = robot.getX() + rx * rCos - ry * rSin;
    double releaseY = robot.getY() + rx * rSin + ry * rCos;

    double horizontalVel = launchMps * ch;
    double verticalVel = launchMps * sh;
    double fieldYaw = robot.getRotation().getRadians() + turretRad;
    double xVel = horizontalVel * Math.cos(fieldYaw);
    double yVel = horizontalVel * Math.sin(fieldYaw);
    ChassisSpeeds fieldSpeeds =
        RobotState.getInstance().getLatestMeasuredFieldRelativeChassisSpeeds();
    if (fieldSpeeds != null) {
      xVel += fieldSpeeds.vxMetersPerSecond;
      yVel += fieldSpeeds.vyMetersPerSecond;
    }

    fuelSim.spawnFuel(
        new Translation3d(releaseX, releaseY, rz), new Translation3d(xVel, yVel, verticalVel));
  }
}
