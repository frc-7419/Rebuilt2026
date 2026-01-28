// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;

public class VisionConstants {
  // AprilTag layout
  public static AprilTagFieldLayout aprilTagLayout =
      AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

  public static String kLimelightFourTable = "limelight-four";
  public static String kLimelightThreeTable = "limelight-three";

  // Camera pose constants (in robot space: [forward, side, up, roll, pitch, yaw])
  public static final double kLimelightFourCameraHeightM = 0.15;
  public static final double kLimelightFourCameraPitchDeg = -15;
  public static final double kLimelightFourCameraYawDeg = -45;
  public static final double[] kLimelightFourCameraPose = {
    0.33,
    -0.33,
    kLimelightFourCameraHeightM,
    0,
    kLimelightFourCameraPitchDeg,
    kLimelightFourCameraYawDeg
  };

  public static final double kLimelightThreeCameraHeightM = 0.15;
  public static final double kLimelightThreeCameraPitchDeg = -15;
  public static final double kLimelightThreeCameraYawDeg = 45;
  public static final double[] kLimelightThreeCameraPose = {
    0.33,
    0.33,
    kLimelightThreeCameraHeightM,
    0,
    kLimelightThreeCameraPitchDeg,
    kLimelightThreeCameraYawDeg
  };
}
