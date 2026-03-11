// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.RobotBase;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
 * (log replay from a file).
 */
public final class Constants {
  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  public static Pose3d turretBasePose = new Pose3d(0.158750, 0.158750, 0.298454, new Rotation3d());
  public static final Transform3d turretToHood =
      new Transform3d(
          new Translation3d(0.252158 - 0.158750, 0.1546565 - 0.158750, 0.431800 - 0.298454),
          new Rotation3d());
  public static Pose3d hoodBasePose = new Pose3d(0.252158, 0.1546565, 0.431800, new Rotation3d());
  public static Pose3d intakeBasePose = new Pose3d(-0.294640, 0.0, 0.193312, new Rotation3d());
  public static Pose3d hopperBasePose = new Pose3d(-0.317817, 0.0, 0.427038, new Rotation3d());
  public static final Distance kHopperMaxExtension = Meters.of(0.301625);

  /** Hub center (blue). */
  public static final Pose2d kHubPoseBlue = new Pose2d(4.620, 4.030, Rotation2d.kZero);

  /** Passing target high field */
  public static final Pose2d kPassingPoseHighBlue = new Pose2d(2.25, 6.25, Rotation2d.kZero);

  /** Passing target low field */
  public static final Pose2d kPassingPoseLowBlue = new Pose2d(2.25, 1.25, Rotation2d.kZero);

  /** Field midpoint */
  public static final double kPassingYThresholdMeters = 4.0;

  /** Hub center as first constraint. */
  public static final double kHubTargetHeightMeters = 54 * 0.0254;

  /** Hub funnel geometry */
  public static final double kHubFunnelRadiusMeters = 0.61; // ~24 in

  /** Height of funnel rim */
  public static final double kHubFunnelHeightMeters = 72 * 0.0254;

  /** Clearance above rim the ball must pass. FunnelClear Z = rim + this. */
  public static final double kHubFunnelClearanceMeters = 0.115;

  /** When true, sim tracks fuel stored (capacity) and only launches if fuelStored > 0. */
  public static final boolean kSimulateFuelCapacity = true;

  /** Max fuel the robot can store when simulating capacity. */
  public static final int kFuelCapacity = 50;

  /**
   * Max intake rate (fuel per second) when simulating capacity; used for canIntake() rate limit.
   */
  public static final double kMaxIntakeRatePerSecond = 50.0;

  public static final int kLimelightMode = 0;
}
