// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.AutoLogOutput;

/**
 * Vision subsystem placeholder for pose estimation using AprilTags.
 *
 * <p>This is a stub implementation that loads the 2026 FRC AprilTag field layout.
 *
 * <p>To integrate with PhotonVision cameras: 1. Install PhotonVision on your coprocessor 2. Update
 * this class to use PhotonVision libraries once a stable version is available 3. Configure camera
 * names and transforms in VisionConstants.java
 */
public class Vision extends SubsystemBase {
  // AprilTag field layout for 2026 FRC game
  private final AprilTagFieldLayout fieldLayout;

  // Robot pose (placeholder)
  private Pose2d robotPose = new Pose2d();

  // Latency tracking
  private double lastFrameTimestamp = 0.0;
  private double latency = 0.0;

  // Alerts
  private final Alert fieldLayoutAlert =
      new Alert("Failed to load AprilTag field layout.", AlertType.kError);

  /**
   * Creates a Vision subsystem stub.
   *
   * <p>This loads the 2026 FRC AprilTag field layout. PhotonVision integration will be added once a
   * stable version is available.
   */
  public Vision() {
    // Load the AprilTag field layout for 2026 FRC game
    try {
      fieldLayout = AprilTagFields.kDefaultField.loadAprilTagLayoutField();
      System.out.println("✓ AprilTag field layout loaded for 2026 FRC game");
    } catch (Exception e) {
      fieldLayoutAlert.set(true);
      System.err.println("✗ Failed to load AprilTag field layout: " + e.getMessage());
      throw new RuntimeException("Failed to load AprilTag field layout", e);
    }
  }

  @Override
  public void periodic() {
    // Vision system is currently a stub - no measurements are generated
    // This will be updated when PhotonVision integration is available
  }

  /**
   * Gets the latest estimated robot pose.
   *
   * <p>Currently returns zero pose. Will be updated with actual vision measurements when
   * PhotonVision integration is available.
   *
   * @return The estimated robot pose
   */
  @AutoLogOutput(key = "Vision/EstimatedPose")
  public Pose2d getPose() {
    return robotPose;
  }

  /**
   * Gets the latest estimated robot pose in 3D.
   *
   * @return The estimated robot pose in 3D
   */
  public Pose3d getPose3d() {
    return new Pose3d(robotPose);
  }

  /**
   * Gets the timestamp of the last vision measurement.
   *
   * @return The timestamp in seconds
   */
  @AutoLogOutput(key = "Vision/LastMeasurementTimestamp")
  public double getLastMeasurementTimestamp() {
    return lastFrameTimestamp;
  }

  /**
   * Gets the latency of the vision measurement.
   *
   * @return The latency in seconds
   */
  @AutoLogOutput(key = "Vision/MeasurementLatency")
  public double getLatency() {
    return latency;
  }

  /**
   * Gets the vision measurement standard deviations for the drive pose estimator.
   *
   * @param distance Distance from the robot to the target tag
   * @return Standard deviations as a matrix
   */
  public Matrix<N3, N1> getVisionMeasurementStdDevs(double distance) {
    // Increase standard deviations as distance increases
    double xyStdDev = 0.1 + (distance * 0.05);
    double thetaStdDev = 0.5 + (distance * 0.1);
    return VecBuilder.fill(xyStdDev, xyStdDev, thetaStdDev);
  }

  /**
   * Gets the approximate measurement standard deviations for general use.
   *
   * @return Standard deviations as a matrix
   */
  public Matrix<N3, N1> getDefaultVisionMeasurementStdDevs() {
    return VecBuilder.fill(0.7, 0.7, 9999999);
  }

  /**
   * Gets the field layout for reference.
   *
   * @return The AprilTag field layout
   */
  public AprilTagFieldLayout getFieldLayout() {
    return fieldLayout;
  }
}
