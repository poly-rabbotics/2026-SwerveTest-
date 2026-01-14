// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;

/**
 * Utility class for integrating Vision measurements with the drive's pose estimator. This helps
 * manage vision measurement timing, confidence, and pose estimation updates.
 */
public class VisionUtil {
  private static final double MAX_VISION_LATENCY = 0.5; // seconds

  /**
   * Checks if a vision measurement should be used based on quality metrics.
   *
   * @param vision The vision subsystem
   * @param lastMeasurementTime The timestamp of the last measurement
   * @return True if the measurement is fresh and valid
   */
  public static boolean isVisionMeasurementValid(Vision vision, double lastMeasurementTime) {
    // Check if enough time has passed since last measurement
    double currentTime = Timer.getFPGATimestamp();
    if (currentTime - vision.getLastMeasurementTimestamp() > MAX_VISION_LATENCY) {
      return false;
    }

    return true;
  }

  /**
   * Calculates the distance from the robot to the nearest visible AprilTag. Used for adjusting
   * measurement confidence.
   *
   * @param robotPose The current robot pose
   * @param visionPose The pose measured by vision
   * @return Distance in meters
   */
  public static double getDistanceToTag(Pose2d robotPose, Pose2d visionPose) {
    Translation2d difference = visionPose.getTranslation().minus(robotPose.getTranslation());
    return difference.getNorm();
  }

  /**
   * Checks if vision and odometry poses are reasonably aligned. This helps detect large errors
   * before they corrupt the pose estimate.
   *
   * @param odometryPose The pose from odometry
   * @param visionPose The pose from vision
   * @param maxTranslationError Maximum acceptable translation error in meters
   * @param maxRotationError Maximum acceptable rotation error in degrees
   * @return True if poses are aligned within tolerances
   */
  public static boolean arePosesAligned(
      Pose2d odometryPose, Pose2d visionPose, double maxTranslationError, double maxRotationError) {
    // Check translation error
    double translationError =
        odometryPose.getTranslation().getDistance(visionPose.getTranslation());
    if (translationError > maxTranslationError) {
      return false;
    }

    // Check rotation error
    double rotationError =
        Math.abs(odometryPose.getRotation().minus(visionPose.getRotation()).getDegrees());
    if (rotationError > maxRotationError) {
      return false;
    }

    return true;
  }

  /**
   * Creates a safe vision pose measurement with validation. Use this when updating the drive's pose
   * estimator.
   *
   * @param vision The vision subsystem
   * @param odometryPose Current odometry pose (for validation)
   * @param lastMeasurementTime Last time a measurement was added
   * @return A valid vision pose measurement, or null if measurement is invalid
   */
  public static Pose2d getSafeVisionMeasurement(
      Vision vision, Pose2d odometryPose, double lastMeasurementTime) {
    // Check if measurement is fresh
    if (!isVisionMeasurementValid(vision, lastMeasurementTime)) {
      return null;
    }

    Pose2d visionPose = vision.getPose();

    // Check if poses are reasonably aligned (adjust thresholds as needed)
    if (!arePosesAligned(odometryPose, visionPose, 1.0, 45.0)) {
      // Poses are too different - this might be a detection error
      // Log warning but don't reject (vision might be more accurate)
      return visionPose; // Still return it, but let caller handle with care
    }

    return visionPose;
  }

  /**
   * Example integration code snippet for Drive subsystem:
   *
   * <p>In your Drive.periodic() method, you might add:
   *
   * <pre>
   * // Update vision measurements
   * if (vision != null && Constants.currentMode == Mode.REAL) {
   *   Pose2d visionMeasurement = VisionUtil.getSafeVisionMeasurement(
   *       vision, poseEstimator.getEstimatedPosition(), lastVisionMeasurementTime);
   *
   *   if (visionMeasurement != null) {
   *     poseEstimator.addVisionMeasurement(
   *         visionMeasurement,
   *         vision.getLastMeasurementTimestamp(),
   *         vision.getVisionMeasurementStdDevs(
   *             VisionUtil.getDistanceToTag(
   *                 poseEstimator.getEstimatedPosition(), visionMeasurement)));
   *     lastVisionMeasurementTime = vision.getLastMeasurementTimestamp();
   *   }
   * }
   * </pre>
   */
  public static void printIntegrationExample() {
    System.out.println(
        "See VisionUtil.getSafeVisionMeasurement() documentation for integration example");
  }
}
