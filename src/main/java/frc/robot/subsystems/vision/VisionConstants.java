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

  public static String kTurretCameraTable = "limelight-turret";
  public static String kSupplementaryCameraTable = "limelight-supplementary";

  // Camera pose constants (in robot space: [forward, side, up, roll, pitch, yaw])
  public static final double kTurretCameraHeightM = 0.5;
  public static final double kTurretCameraPitchDeg = 0.0;
  public static final double[] kTurretCameraPose = {
    0.0, 0.0, kTurretCameraHeightM, 0.0, kTurretCameraPitchDeg, 0.0
  };

  public static final double kSupplementaryCameraHeightM = 0.3;
  public static final double kSupplementaryCameraPitchDeg = -10.0;
  public static final double kSupplementaryCameraRollDeg = 0.0;
  public static final double[] kSupplementaryCameraPose = {
    0.0, 0.0, kSupplementaryCameraHeightM,
    kSupplementaryCameraRollDeg, kSupplementaryCameraPitchDeg, 0.0
  };
}
