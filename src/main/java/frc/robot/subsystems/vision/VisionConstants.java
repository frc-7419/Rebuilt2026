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
  public static final double kLimelightFourCameraForwardM = 0.0877665;
  public static final double kLimelightFourCameraSideM = 0.2018325;
  public static final double kLimelightFourCameraHeightM = 0.498363;
  public static final double kLimelightFourCameraRollDeg = 0;
  public static final double kLimelightFourCameraPitchDeg = 10;
  public static final double kLimelightFourCameraYawDeg = 15;
  public static final double[] kLimelightFourCameraPose = {
    kLimelightFourCameraForwardM,
    kLimelightFourCameraSideM,
    kLimelightFourCameraHeightM,
    kLimelightFourCameraRollDeg,
    kLimelightFourCameraPitchDeg,
    kLimelightFourCameraYawDeg
  };

  public static final double kLimelightThreeCameraForwardM = 0.060;
  public static final double kLimelightThreeCameraSideM = 0.281;
  public static final double kLimelightThreeCameraHeightM = 0.498376;
  public static final double kLimelightThreeCameraPitchDeg = 10;
  public static final double kLimelightThreeCameraYawDeg = -55;
  public static final double[] kLimelightThreeCameraPose = {
    kLimelightThreeCameraForwardM,
    kLimelightThreeCameraSideM,
    kLimelightThreeCameraHeightM,
    0,
    kLimelightThreeCameraPitchDeg,
    kLimelightThreeCameraYawDeg
  };
}
