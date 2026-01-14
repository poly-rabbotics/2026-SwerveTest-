// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.vision;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;

/**
 * Vision subsystem configuration constants. Update these values based on your camera positions and
 * game field for 2026 FRC season.
 */
public class VisionConstants {
  // Camera names as configured in PhotonVision
  public static final String FRONT_CAMERA_NAME =
      "DriveLight"; // Update to match your front camera name
  public static final String BACK_CAMERA_NAME = null; // Set to camera name if using back camera

  // Transform from robot center to front camera
  // These values need to be measured from your robot
  // Forward/Back (positive = forward), Left/Right (positive = left), Up/Down (positive = up)
  public static final Transform3d ROBOT_TO_FRONT_CAMERA =
      new Transform3d(
          new Translation3d(Inches.of(6.0), Inches.of(0.0), Inches.of(12.0)),
          new Rotation3d(0, Math.toRadians(30), 0));

  // Transform from robot center to back camera (if using one)
  public static final Transform3d ROBOT_TO_BACK_CAMERA =
      new Transform3d(
          new Translation3d(Inches.of(-6.0), Inches.of(0.0), Inches.of(12.0)),
          new Rotation3d(0, Math.toRadians(30), Math.PI));
}
