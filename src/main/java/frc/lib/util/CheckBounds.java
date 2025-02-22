package frc.lib.util;

import static frc.robot.Constants.VisionConstants.tagIDToFieldElement;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import frc.lib.util.FieldPose.FieldElement;
import java.util.List;

public class CheckBounds {
  public static FieldElement nearestElement(Pose2d pose) {
    List<AprilTag> tags =
        AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded).getTags();
    FieldElement nearestElement = null;
    double nearestDistance = Double.MAX_VALUE;

    for (AprilTag tag : tags) {
      if (tag.pose.toPose2d().getTranslation().getDistance(pose.getTranslation())
          < nearestDistance) {
        nearestDistance = tag.pose.toPose2d().getTranslation().getDistance(pose.getTranslation());
        nearestElement = tagIDToFieldElement.get(tag.ID);
      }
    }
    return nearestElement;
  }
}
