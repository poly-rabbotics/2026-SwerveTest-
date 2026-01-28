// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;

public class Vision extends SubsystemBase {
  private final VisionIO[] visionIOs;
  private final VisionIOInputsAutoLogged[] inputs;

  // 2026 FRC Reefscape AprilTag positions (in meters, field frame)
  // Blue alliance targets
  private static final Translation2d BLUE_REEF_CENTER = new Translation2d(2.24, 4.11);
  private static final Translation2d BLUE_REEF_LEFT = new Translation2d(1.41, 5.53);
  private static final Translation2d BLUE_REEF_RIGHT = new Translation2d(1.41, 2.69);

  // Red alliance targets (mirrored across field)
  private static final Translation2d RED_REEF_CENTER = new Translation2d(14.26, 4.11);
  private static final Translation2d RED_REEF_LEFT = new Translation2d(15.09, 2.69);
  private static final Translation2d RED_REEF_RIGHT = new Translation2d(15.09, 5.53);

  private int targetAprilTagId = -1; // -1 means track any tag

  public Vision(VisionIO... visionIOs) {
    this.visionIOs = visionIOs;
    this.inputs = new VisionIOInputsAutoLogged[visionIOs.length];
    for (int i = 0; i < visionIOs.length; i++) {
      inputs[i] = new VisionIOInputsAutoLogged();
    }
  }

  @Override
  public void periodic() {
    // Update inputs from all cameras
    for (int i = 0; i < visionIOs.length; i++) {
      visionIOs[i].updateInputs(inputs[i]);
      Logger.processInputs("Vision/" + visionIOs[i].getName(), inputs[i]);
    }
    
    // Log currently tracked AprilTag ID
    Logger.recordOutput("Vision/TargetAprilTagId", targetAprilTagId);
  }

  /**
   * Returns vision measurements for pose estimation. Should be called after odometry updates.
   *
   * @return Array of vision measurements with pose, timestamp, and standard deviations
   */
  public VisionMeasurement[] getVisionMeasurements() {
    int measurementCount = 0;
    for (VisionIOInputsAutoLogged input : inputs) {
      if (input.hasTarget && input.numTags > 0) {
        measurementCount++;
      }
    }

    VisionMeasurement[] measurements = new VisionMeasurement[measurementCount];
    int index = 0;
    for (VisionIOInputsAutoLogged input : inputs) {
      if (input.hasTarget && input.numTags > 0) {
        // Calculate standard deviations based on distance and number of tags
        double xyStdDev = 0.01 + (0.05 * input.averageTagDistance);
        double thetaStdDev = 0.01 + (0.1 * input.averageTagDistance);

        // More tags = more confidence, reduce standard deviation
        if (input.numTags > 1) {
          xyStdDev /= Math.sqrt(input.numTags);
          thetaStdDev /= Math.sqrt(input.numTags);
        }

        measurements[index] =
            new VisionMeasurement(input.robotPose, input.timestamp, xyStdDev, thetaStdDev);
        index++;
      }
    }

    return measurements;
  }

  /** Returns the current robot pose from the best available camera */
  public Pose2d getRobotPose() {
    for (VisionIOInputsAutoLogged input : inputs) {
      if (input.hasTarget) {
        return input.robotPose;
      }
    }
    return new Pose2d();
  }

  /** Returns whether any camera has a valid target */
  public boolean hasTarget() {
    for (VisionIOInputsAutoLogged input : inputs) {
      if (input.hasTarget) {
        return true;
      }
    }
    return false;
  }

  /**
   * Sets the target AprilTag ID to track. 
   * Set to -1 to track any visible tag.
   * 
   * @param tagId The AprilTag ID to track, or -1 for any tag
   */
  public void setTargetAprilTagId(int tagId) {
    targetAprilTagId = tagId;
  }

  /**
   * Gets the currently tracked AprilTag ID.
   * 
   * @return The target AprilTag ID, or -1 if tracking any tag
   */
  public int getTargetAprilTagId() {
    return targetAprilTagId;
  }

  /**
   * Returns whether the specified AprilTag is currently visible.
   * 
   * @param tagId The AprilTag ID to check
   * @return True if the tag is visible in any camera
   */
  public boolean isTagVisible(int tagId) {
    for (VisionIOInputsAutoLogged input : inputs) {
      if (input.hasTarget && input.targetId == tagId) {
        return true;
      }
    }
    return false;
  }

  /**
   * Returns whether the currently set target AprilTag is visible.
   * 
   * @return True if the target tag is visible, or if tracking any tag and any tag is visible
   */
  public boolean isTargetVisible() {
    if (targetAprilTagId == -1) {
      return hasTarget();
    }
    return isTagVisible(targetAprilTagId);
  }

  /**
   * Gets the distance to the currently tracked AprilTag.
   * 
   * @return Distance in meters, or 0.0 if tag not visible
   */
  public double getDistanceToTarget() {
    for (VisionIOInputsAutoLogged input : inputs) {
      if (input.hasTarget) {
        // If tracking specific tag, only return distance if it matches
        if (targetAprilTagId == -1 || input.targetId == targetAprilTagId) {
          return input.targetDistance;
        }
      }
    }
    return 0.0;
  }

  /**
   * Gets the horizontal offset (tx) to the currently tracked AprilTag.
   * 
   * @return Horizontal offset in degrees, or 0.0 if tag not visible
   */
  public double getTargetTx() {
    for (VisionIOInputsAutoLogged input : inputs) {
      if (input.hasTarget) {
        if (targetAprilTagId == -1 || input.targetId == targetAprilTagId) {
          return input.tx;
        }
      }
    }
    return 0.0;
  }

  /**
   * Gets the vertical offset (ty) to the currently tracked AprilTag.
   * 
   * @return Vertical offset in degrees, or 0.0 if tag not visible
   */
  public double getTargetTy() {
    for (VisionIOInputsAutoLogged input : inputs) {
      if (input.hasTarget) {
        if (targetAprilTagId == -1 || input.targetId == targetAprilTagId) {
          return input.ty;
        }
      }
    }
    return 0.0;
  }

  /**
   * Returns distance to the reef center scoring position based on alliance.
   * Updates every robot loop after odometry is updated.
   */
  public double distanceToReefCenter(Pose2d robotPose) {
    Translation2d target = getReefCenter();
    return robotPose.getTranslation().getDistance(target);
  }

  /**
   * Returns distance to the left reef scoring zone based on alliance.
   * Updates every robot loop after odometry is updated.
   */
  public double distanceToReefLeft(Pose2d robotPose) {
    Translation2d target = getReefLeft();
    return robotPose.getTranslation().getDistance(target);
  }

  /**
   * Returns distance to the right reef scoring zone based on alliance.
   * Updates every robot loop after odometry is updated.
   */
  public double distanceToReefRight(Pose2d robotPose) {
    Translation2d target = getReefRight();
    return robotPose.getTranslation().getDistance(target);
  }

  /** Returns the reef center position in field frame based on current alliance */
  public Translation2d getReefCenter() {
    Optional<Alliance> alliance = DriverStation.getAlliance();
    if (alliance.isPresent()) {
      if (alliance.get() == Alliance.Red) {
        return RED_REEF_CENTER;
      } else {
        return BLUE_REEF_CENTER;
      }
    }
    return BLUE_REEF_CENTER; // Default to blue
  }

  /** Returns the left reef position in field frame based on current alliance */
  public Translation2d getReefLeft() {
    Optional<Alliance> alliance = DriverStation.getAlliance();
    if (alliance.isPresent()) {
      if (alliance.get() == Alliance.Red) {
        return RED_REEF_LEFT;
      } else {
        return BLUE_REEF_LEFT;
      }
    }
    return BLUE_REEF_LEFT; // Default to blue
  }

  /** Returns the right reef position in field frame based on current alliance */
  public Translation2d getReefRight() {
    Optional<Alliance> alliance = DriverStation.getAlliance();
    if (alliance.isPresent()) {
      if (alliance.get() == Alliance.Red) {
        return RED_REEF_RIGHT;
      } else {
        return BLUE_REEF_RIGHT;
      }
    }
    return BLUE_REEF_RIGHT; // Default to blue
  }

  /** Vision measurement container class */
  public static class VisionMeasurement {
    public final Pose2d pose;
    public final double timestamp;
    public final double xyStdDev;
    public final double thetaStdDev;

    public VisionMeasurement(Pose2d pose, double timestamp, double xyStdDev, double thetaStdDev) {
      this.pose = pose;
      this.timestamp = timestamp;
      this.xyStdDev = xyStdDev;
      this.thetaStdDev = thetaStdDev;
    }
  }
}
