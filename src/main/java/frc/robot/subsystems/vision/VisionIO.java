

package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import org.littletonrobotics.junction.AutoLog;

public interface VisionIO {
  @AutoLog
  public static class VisionIOInputs {
    public boolean connected = false;
    public Pose2d robotPose = new Pose2d();
    public double timestamp = 0.0;
    public boolean hasTarget = false;
    public int targetId = -1;
    public double targetDistance = 0.0;
    public double averageTagDistance = 0.0;
    public int numTags = 0;
    public double tx = 0.0; // Horizontal offset from crosshair to target
    public double ty = 0.0; // Vertical offset from crosshair to target
    public double ta = 0.0; // Target area (0-100% of image)
  }

  public default void updateInputs(VisionIOInputs inputs) {}

  public default String getName() {
    return "";
  }
}
