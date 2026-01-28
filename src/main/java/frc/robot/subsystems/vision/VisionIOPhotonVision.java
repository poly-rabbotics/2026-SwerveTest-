// 

package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class VisionIOPhotonVision implements VisionIO {
  private final PhotonCamera camera;
  private final PhotonPoseEstimator poseEstimator;
  private final String name;
  private AprilTagFieldLayout fieldLayout;

  /**
   * Creates a new VisionIOPhotonVision.
   *
   * @param cameraName The name of the PhotonVision camera
   * @param robotToCamera The transform from the robot center to the camera (in 3D space)
   */
  public VisionIOPhotonVision(String cameraName, Transform3d robotToCamera) {
    this.name = cameraName;
    this.camera = new PhotonCamera(cameraName);

    
    try {
      
      fieldLayout = AprilTagFieldLayout.loadFromResource("/edu/wpi/first/apriltag/2026-rebuilt.json");
    } catch (Exception e) {
      System.err.println("Failed to load 2026 Rebuilt AprilTag field layout!");
      e.printStackTrace();
      fieldLayout = null;
    }

    // Create pose estimator using multi-tag strategy
    if (fieldLayout != null) {
      poseEstimator = new PhotonPoseEstimator(fieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, robotToCamera);
      poseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
    } else {
      poseEstimator = null;
    }
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    // Get the latest result from PhotonVision
    PhotonPipelineResult result = camera.getLatestResult();

    inputs.connected = camera.isConnected();
    inputs.hasTarget = result.hasTargets();

    if (inputs.hasTarget) {
      // Get the best target (usually the one with lowest ambiguity)
      PhotonTrackedTarget bestTarget = result.getBestTarget();

      // Get target info
      inputs.targetId = bestTarget.getFiducialId();
      inputs.tx = bestTarget.getYaw(); // Horizontal offset in degrees
      inputs.ty = bestTarget.getPitch(); // Vertical offset in degrees
      inputs.ta = bestTarget.getArea(); // Target area percentage

      // Calculate distance to target using the pose ambiguity
      Transform3d targetTransform = bestTarget.getBestCameraToTarget();
      inputs.targetDistance = targetTransform.getTranslation().getNorm();

      // Get robot pose estimate if we have the field layout
      if (poseEstimator != null) {
        Optional<EstimatedRobotPose> estimatedPose = poseEstimator.update(result);
        if (estimatedPose.isPresent()) {
          EstimatedRobotPose pose = estimatedPose.get();
          
          // Convert 3D pose to 2D pose for the drive subsystem
          inputs.robotPose = pose.estimatedPose.toPose2d();
          
          // PhotonVision timestamps are in seconds (already FPGA time)
          inputs.timestamp = pose.timestampSeconds;
          
          // Number of tags used in the estimate
          inputs.numTags = pose.targetsUsed.size();
          
          // Calculate average distance to all tags used
          double totalDistance = 0.0;
          for (PhotonTrackedTarget target : pose.targetsUsed) {
            totalDistance += target.getBestCameraToTarget().getTranslation().getNorm();
          }
          inputs.averageTagDistance = inputs.numTags > 0 ? totalDistance / inputs.numTags : 0.0;
        } else {
          // Pose estimation failed, clear pose data but keep target info
          inputs.robotPose = new Pose2d();
          inputs.timestamp = result.getTimestampSeconds();
          inputs.numTags = 0;
          inputs.averageTagDistance = 0.0;
        }
      }
    } else {
      // No target detected
      inputs.targetId = -1;
      inputs.targetDistance = 0.0;
      inputs.tx = 0.0;
      inputs.ty = 0.0;
      inputs.ta = 0.0;
      inputs.numTags = 0;
      inputs.averageTagDistance = 0.0;
    }
  }

  @Override
  public String getName() {
    return name;
  }
}
