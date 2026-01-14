// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.vision.Vision;

/**
 * Example command showing how to use the vision subsystem for logging and debugging. This logs
 * vision pose data to SmartDashboard.
 */
public class VisionDebugCommand extends Command {
  private final Vision vision;
  private final Drive drive;

  /**
   * Creates a new VisionDebugCommand.
   *
   * @param vision The vision subsystem
   * @param drive The drive subsystem (for pose estimation integration)
   */
  public VisionDebugCommand(Vision vision, Drive drive) {
    this.vision = vision;
    this.drive = drive;
  }

  @Override
  public void execute() {
    // Log vision pose
    Pose2d visionPose = vision.getPose();
    SmartDashboard.putNumber("Vision X", visionPose.getX());
    SmartDashboard.putNumber("Vision Y", visionPose.getY());
    SmartDashboard.putNumber("Vision Rotation", visionPose.getRotation().getDegrees());

    // Log camera status
    // TODO: Replace with correct method names from Vision subsystem
    // SmartDashboard.putBoolean("Front Camera Connected", vision.isFrontCameraConnected());
    // SmartDashboard.putBoolean("Back Camera Connected", vision.isBackCameraConnected());

    // Log measurement latency
    SmartDashboard.putNumber("Vision Latency", vision.getLatency());
    SmartDashboard.putNumber("Vision Timestamp", vision.getLastMeasurementTimestamp());

    // Log drive pose for comparison
    SmartDashboard.putNumber("Drive X", drive.getPose().getX());
    SmartDashboard.putNumber("Drive Y", drive.getPose().getY());
    SmartDashboard.putNumber("Drive Rotation", drive.getPose().getRotation().getDegrees());
  }

  @Override
  public boolean isFinished() {
    // This command should run indefinitely
    return false;
  }
}
