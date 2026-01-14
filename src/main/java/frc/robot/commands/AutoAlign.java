// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.vision.Vision;
import java.util.function.DoubleSupplier;

/**
 * AutoAlign command allows the driver to control the robot's translation with joysticks while the
 * robot automatically rotates to face a target AprilTag.
 *
 * <p>The rotation is controlled by a PID controller that tries to keep the robot's heading aligned
 * with the target tag, while translation is controlled by the joystick inputs.
 */
public class AutoAlign extends Command {
  private final Drive drive;
  private final Vision vision;
  private final DoubleSupplier translationXSupplier;
  private final DoubleSupplier translationYSupplier;
  private final int targetTagId;

  // PID controller for rotation alignment
  private final PIDController rotationPID;

  // Configuration constants
  private static final double ROTATION_P = 0.1; // Proportional gain
  private static final double ROTATION_I = 0.0; // Integral gain
  private static final double ROTATION_D = 0.05; // Derivative gain
  private static final double MAX_ROTATION_SPEED = 2.0; // rad/s
  private static final double TOLERANCE_DEGREES = 2.0; // Degrees of tolerance for alignment

  /**
   * Creates an AutoAlign command.
   *
   * @param drive The drive subsystem
   * @param vision The vision subsystem
   * @param translationXSupplier Supplier for forward/backward translation (-1 to 1)
   * @param translationYSupplier Supplier for left/right translation (-1 to 1)
   * @param targetTagId The AprilTag ID to align to
   */
  public AutoAlign(
      Drive drive,
      Vision vision,
      DoubleSupplier translationXSupplier,
      DoubleSupplier translationYSupplier,
      int targetTagId) {
    this.drive = drive;
    this.vision = vision;
    this.translationXSupplier = translationXSupplier;
    this.translationYSupplier = translationYSupplier;
    this.targetTagId = targetTagId;

    // Initialize PID controller for rotation
    this.rotationPID = new PIDController(ROTATION_P, ROTATION_I, ROTATION_D);
    this.rotationPID.enableContinuousInput(-Math.PI, Math.PI);
    this.rotationPID.setTolerance(Math.toRadians(TOLERANCE_DEGREES));

    // Require the drive subsystem (prevents multiple commands from using it)
    addRequirements(drive);
  }

  @Override
  public void initialize() {
    // Initialize the PID controller
    rotationPID.reset();
    System.out.println("AutoAlign initialized - targeting tag " + targetTagId);
  }

  @Override
  public void execute() {
    // Get joystick inputs for translation
    double translationX = translationXSupplier.getAsDouble();
    double translationY = translationYSupplier.getAsDouble();

    // Get robot's current pose
    var robotPose = drive.getPose();
    var targetRotation = robotPose.getRotation();

    // Get the target tag's pose from the field layout
    var fieldLayout = vision.getFieldLayout();
    var targetTagOptional = fieldLayout.getTagPose(targetTagId);

    if (targetTagOptional.isPresent()) {
      // Calculate the desired rotation to face the target tag
      var targetTagPose = targetTagOptional.get();
      var targetTagPosition = targetTagPose.getTranslation().toTranslation2d();
      var robotPosition = robotPose.getTranslation();

      // Calculate the angle from robot to tag
      var delta = targetTagPosition.minus(robotPosition);
      var desiredRotation = new Rotation2d(delta.getX(), delta.getY());

      // Use PID controller to calculate rotation velocity
      double rotationVelocity =
          rotationPID.calculate(targetRotation.getRadians(), desiredRotation.getRadians());

      // Clamp rotation velocity to max speed
      rotationVelocity =
          Math.max(-MAX_ROTATION_SPEED, Math.min(MAX_ROTATION_SPEED, rotationVelocity));

      // Log debug information
      SmartDashboard.putNumber(
          "AutoAlign/DesiredRotation", Math.toDegrees(desiredRotation.getRadians()));
      SmartDashboard.putNumber(
          "AutoAlign/CurrentRotation", Math.toDegrees(targetRotation.getRadians()));
      SmartDashboard.putNumber(
          "AutoAlign/RotationError",
          Math.toDegrees(desiredRotation.minus(targetRotation).getRadians()));
      SmartDashboard.putNumber("AutoAlign/RotationVelocity", rotationVelocity);
      SmartDashboard.putBoolean("AutoAlign/AtTarget", rotationPID.atSetpoint());

      // Create chassis speeds with joystick translation and PID rotation
      ChassisSpeeds speeds =
          ChassisSpeeds.fromFieldRelativeSpeeds(
              translationX * drive.getMaxLinearSpeedMetersPerSec(),
              translationY * drive.getMaxLinearSpeedMetersPerSec(),
              rotationVelocity,
              targetRotation);

      // Send speeds to the drive
      drive.runVelocity(speeds);
    } else {
      // Tag not found, just use joystick control without rotation
      System.err.println("AutoAlign: Target tag " + targetTagId + " not found in field layout");

      ChassisSpeeds speeds =
          ChassisSpeeds.fromFieldRelativeSpeeds(
              translationX * drive.getMaxLinearSpeedMetersPerSec(),
              translationY * drive.getMaxLinearSpeedMetersPerSec(),
              0.0,
              targetRotation);

      drive.runVelocity(speeds);
    }
  }

  @Override
  public void end(boolean interrupted) {
    // Stop the robot
    drive.runVelocity(new ChassisSpeeds());
    System.out.println("AutoAlign ended" + (interrupted ? " (interrupted)" : ""));
  }

  @Override
  public boolean isFinished() {
    // Command runs until interrupted (e.g., when button is released)
    return false;
  }

  /**
   * Checks if the robot is aligned to the target tag within the tolerance.
   *
   * @return True if aligned, false otherwise
   */
  public boolean isAligned() {
    return rotationPID.atSetpoint();
  }
}
