package frc.lib.util;

import static edu.wpi.first.units.Units.*;
import static frc.robot.Constants.FieldConstants.*;
import static frc.robot.Constants.VisionConstants.tagIDToFieldElement;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import frc.lib.util.FieldPose.FieldElement;
import frc.lib.util.FieldPose.Offset;
import frc.robot.subsystems.Drivetrain.Swerve;
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

  public static int nearestTag(Pose2d pose) {
    List<AprilTag> tags =
        AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded).getTags();
    int nearestTag = 0;
    double nearestDistance = Double.MAX_VALUE;

    for (AprilTag tag : tags) {
      if (tag.pose.toPose2d().getTranslation().getDistance(pose.getTranslation())
          < nearestDistance) {
        nearestDistance = tag.pose.toPose2d().getTranslation().getDistance(pose.getTranslation());
        nearestTag = tag.ID;
      }
    }
    System.out.println("Nearest tag: " + nearestTag);
    return nearestTag;
  }

  public static Pose2d getPose2DFromFieldPose(Swerve swerve, FieldPose targetPose) {
    int target = nearestTag(swerve.getPose());
    targetPose.fieldElement = CheckBounds.nearestElement(swerve.getPose());
    Pose2d tagPose =
        AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded)
            .getTagPose(target)
            .get()
            .toPose2d();
    // return tagPose;
    Rotation2d tagHeading = tagPose.getRotation();
    double bumperSize = 17.5;
    Pose2d poseToDrive =
        tagPose.transformBy(
            new Transform2d(
               Inches.of(bumperSize).in(Meters),
                0,
                new Rotation2d(0)));
    // flip the pose
    if (FieldPose.fieldElementIsReef(targetPose.fieldElement) && targetPose.offset == Offset.LEFT) {
      tagHeading = tagHeading.rotateBy(Rotation2d.kCW_90deg);
      poseToDrive =
          poseToDrive.transformBy(
              new Transform2d(
                  0,
                  reefLeftPoseOffset.in(Meters),
                  Rotation2d.fromDegrees(0)));
    }
    if (FieldPose.fieldElementIsReef(targetPose.fieldElement)
        && targetPose.offset == Offset.RIGHT) {
      tagHeading = tagHeading.rotateBy(Rotation2d.kCW_90deg);
      poseToDrive =
          poseToDrive.transformBy(
              new Transform2d(
                  0,
                  reefLeftPoseOffset.in(Meters),
                  Rotation2d.fromDegrees(0)));
    }
    if (FieldPose.fieldElementIsReef(targetPose.fieldElement)) {
      return new Pose2d(poseToDrive.getX(), poseToDrive.getY(), Rotation2d.fromDegrees(poseToDrive.getRotation().getDegrees() - 180));
    }
    else if (FieldPose.fieldElementIsHPStation(targetPose.fieldElement) {
      return new Pose2d(poseToDrive.getX(), poseToDrive.getY(), Rotation2d.fromDegrees(poseToDrive.getRotation().getDegrees() - 90));
    }
    return poseToDrive;
  }
}
