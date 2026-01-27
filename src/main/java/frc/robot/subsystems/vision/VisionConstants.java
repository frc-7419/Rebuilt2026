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

  public static String kLeftCameraTable = "limelight-left";
  public static String kRightCameraTable = "limelight-right";

  // Camera pose constants (in robot space: [forward, side, up, roll, pitch, yaw])
  public static final double kLeftCameraHeightM = 0.15;
  public static final double kLeftCameraPitchDeg = -15;
  public static final double kLeftCameraYawDeg = -45;
  public static final double[] kLeftCameraPose = {
    0.33, -0.33, kLeftCameraHeightM, 0, kLeftCameraPitchDeg, kLeftCameraYawDeg
  };

  public static final double kRightCameraHeightM = 0.15;
  public static final double kRightCameraPitchDeg = -15;
  public static final double kRightCameraYawDeg = 45;
  public static final double[] kRightCameraPose = {
    0.33, 0.33, kRightCameraHeightM, 0, kRightCameraPitchDeg, kRightCameraYawDeg
  };
}
