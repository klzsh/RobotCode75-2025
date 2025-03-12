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
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.lib.util.FieldPose.FieldElement;
import frc.lib.util.FieldPose.Offset;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.Drivetrain.Swerve;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

class IDVectorPair {
  public int id;
  public Translation2d vector;

  public IDVectorPair(int id, Translation2d vector) {
    this.id = id;
    this.vector = vector;
  }

  public String toString() {
    return id + ": " + vector.getNorm();
  }
}

// screw peddie screw peddie screw peddie screw peddie screw peddie screw peddie screw peddie screw peddie screw peddie screw peddie screw peddie 
public class PeddieBounds {

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

  public static Pose2d getNearestFieldPose2d(Swerve swerve, FieldPose targetPose) {
    targetPose.fieldElement = nearestElement(swerve.getPose());
    return fieldElementToPose2d(swerve, targetPose);
  }

  public static Pose2d fieldElementToPose2d(Swerve swerve, FieldPose targetPose) {
    int targetTag =
        targetPose.alliance == Alliance.Blue
            ? blueTags.get(targetPose.fieldElement)
            : redTags.get(targetPose.fieldElement);
    Pose2d tagPose =
        AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded)
            .getTagPose(targetTag)
            .get()
            .toPose2d();
    // return tagPose;
    Rotation2d tagHeading = tagPose.getRotation();
    double bumperSize = 18.5;
    Pose2d poseToDrive =
        tagPose.transformBy(
            new Transform2d(Inches.of(bumperSize).in(Meters), 0, new Rotation2d(0)));
    // flip the pose
    if (FieldPose.fieldElementIsReef(targetPose.fieldElement) && targetPose.offset == Offset.LEFT) {
      tagHeading = tagHeading.rotateBy(Rotation2d.kCW_90deg);
      poseToDrive =
          poseToDrive.transformBy(
              new Transform2d(0, reefLeftPoseOffset.in(Meters), Rotation2d.fromDegrees(0)));
    }
    if (FieldPose.fieldElementIsReef(targetPose.fieldElement)
        && targetPose.offset == Offset.RIGHT) {
      tagHeading = tagHeading.rotateBy(Rotation2d.kCW_90deg);
      poseToDrive =
          poseToDrive.transformBy(
              new Transform2d(0, reefRightPoseOffset.in(Meters), Rotation2d.fromDegrees(0)));
    }
    if (FieldPose.fieldElementIsReef(targetPose.fieldElement)) {
      return new Pose2d(
          poseToDrive.getX(),
          poseToDrive.getY(),
          Rotation2d.fromDegrees(poseToDrive.getRotation().getDegrees() - 180));
    } else if (FieldPose.fieldElementIsHPStation(targetPose.fieldElement)) {
      return new Pose2d(
          poseToDrive.getX(),
          poseToDrive.getY(),
          Rotation2d.fromDegrees(poseToDrive.getRotation().getDegrees() - 90));
    }
    return poseToDrive;
  }

  private static Swerve m_Swerve;
  private static List<Translation2d> badHexagonPoints;

  private static double cosineSimilarity(Translation2d a, Translation2d b) {
    return (a.getX() * b.getX() + a.getY() * b.getY()) / (a.getNorm() * b.getNorm());
  }

  public static void init(Swerve swerve) {
    m_Swerve = swerve;
    Translation2d reefCenter;
    AprilTagFieldLayout field = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);
    if (DriverStation.getAlliance().isEmpty()
        || DriverStation.getAlliance().get() == DriverStation.Alliance.Blue) {
      // BLUE
      Pose2d tag18 = field.getTagPose(18).get().toPose2d();
      Pose2d tag21 = field.getTagPose(21).get().toPose2d();
      reefCenter =
          new Translation2d((tag18.getX() + tag21.getX()) / 2, (tag18.getY() + tag21.getY()) / 2);
    } else {
      // RED
      Pose2d tag7 = field.getTagPose(7).get().toPose2d();
      Pose2d tag10 = field.getTagPose(10).get().toPose2d();
      reefCenter =
          new Translation2d((tag7.getX() + tag10.getX()) / 2, (tag7.getY() + tag10.getY()) / 2);
    }

    /*
     * center to tag = 32.75
     * divide reef hexagon into 6 hexagons: each are equilateral
     *
     * /|
     * / |
     * / |
     * / |
     * a / | 32.75
     * / |
     * / |
     * / 60d |
     * ----------
     * sin(60d) = sqrt(3)/2 = 32.75/a
     * => a = 2*32.75 / sqrt(3) = 37.816
     */

    double reefCornerToCenter = Units.inchesToMeters(2 * 32.75 / Math.sqrt(3));
    reefCornerToCenter += 1.2;

    badHexagonPoints = new ArrayList<>();
    for (int i = 0; i < 360; i += 60) {
      badHexagonPoints.add(
          new Translation2d(
              reefCenter.getX() + reefCornerToCenter * Math.cos(Math.toRadians(i)),
              reefCenter.getY() + reefCornerToCenter * Math.sin(Math.toRadians(i))));
    }
  }

  private static double angle2d(double x1, double y1, double x2, double y2) {
    double dtheta, theta1, theta2;
    theta1 = Math.atan2(y1, x1);
    theta2 = Math.atan2(y2, x2);
    dtheta = theta2 - theta1;
    while (dtheta > Math.PI) dtheta -= 2 * Math.PI;
    while (dtheta < -Math.PI) dtheta += 2 * Math.PI;
    return dtheta;
  }

  private static boolean insideBadHexagon(Translation2d p) {
    double angle = 0;
    Translation2d p1 = new Translation2d(0, 0);
    Translation2d p2 = new Translation2d(0, 0);

    for (int i = 0; i < 6; i++) {
      p1 =
          new Translation2d(
              badHexagonPoints.get(i).getX() - p.getX(), badHexagonPoints.get(i).getY() - p.getY());

      p2 =
          new Translation2d(
              badHexagonPoints.get((i + 1) % 6).getX() - p.getX(),
              badHexagonPoints.get((i + 1) % 6).getY() - p.getY());

      angle += angle2d(p1.getX(), p1.getY(), p2.getX(), p2.getY());
    }

    return Math.abs(angle) >= Math.PI;
  }

  public static boolean insideBadHexagon(Pose2d p) {
    return insideBadHexagon(new Translation2d(p.getX(), p.getY()));
  }

  public static FieldElement getReefElement() {
    // calculate current odometry pose
    Translation2d odometryPose = m_Swerve.getPose().getTranslation();
    List<IDVectorPair> robotToTag = new ArrayList<>();
    AprilTagFieldLayout field = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);

    if (DriverStation.getAlliance().isEmpty()
        || DriverStation.getAlliance().get() == DriverStation.Alliance.Blue) {
      // BLUE:
      for (int i = 17; i <= 22; i++) {
        Translation2d tagPose = field.getTagPose(i).get().toPose2d().getTranslation();
        robotToTag.add(new IDVectorPair(i, tagPose.minus(odometryPose)));
      }
    } else {
      // RED:
      for (int i = 6; i <= 11; i++) {
        Translation2d tagPose = field.getTagPose(i).get().toPose2d().getTranslation();
        robotToTag.add(new IDVectorPair(i, tagPose.minus(odometryPose)));
      }
    }

    // sort list in ascending order of vector magnitude / robot distance to tag
    Collections.sort(
        robotToTag, (o1, o2) -> (((Double) o1.vector.getNorm()).compareTo(o2.vector.getNorm())));

    // closest and second closest tag IDs
    int tag0id = robotToTag.get(0).id;
    int tag1id = robotToTag.get(1).id;

    boolean isInBadHexagon = insideBadHexagon(odometryPose);

    double tag0neededAngle = FieldConstants.tagToHeadingMap.get(tag0id);
    double tag0gyroError = Math.abs(m_Swerve.getRotationDegrees() - tag0neededAngle);

    if (isInBadHexagon)
      return tag0gyroError < 30.0 ? VisionConstants.tagIDToFieldElement.get(tag0id) : null;

    if (robotToTag.get(1).vector.getNorm() - robotToTag.get(0).vector.getNorm() >= 0.25)
      return tag0gyroError < 45.0 ? VisionConstants.tagIDToFieldElement.get(tag0id) : null;

    ChassisSpeeds speeds = m_Swerve.getChassisSpeeds();
    Translation2d robotMovement =
        new Translation2d(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);

    int bestTag;
    if (robotMovement.getNorm() == 0) bestTag = tag0id;
    else {
      double similar0 = cosineSimilarity(robotToTag.get(0).vector, robotMovement);
      double similar1 = cosineSimilarity(robotToTag.get(1).vector, robotMovement);
      bestTag = similar0 >= similar1 ? tag0id : tag1id;
    }

    double bestTagNeededAngle = FieldConstants.tagToHeadingMap.get(bestTag);
    return Math.abs(m_Swerve.getRotationDegrees() - bestTagNeededAngle) < 45.0
        ? VisionConstants.tagIDToFieldElement.get(bestTag)
        : null;
  }

  private static final double hpThreshold =
      0.5; // section in the middle where bound is determined by vel

  public static FieldElement getHPElement() {
    Translation2d odometryPose = m_Swerve.getPose().getTranslation();
    AprilTagFieldLayout field = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);

    if (DriverStation.getAlliance().isEmpty()
        || DriverStation.getAlliance().get() == Alliance.Blue) {
      // BLUE:
      double distToT =
          field.getTagPose(13).get().toPose2d().getTranslation().getDistance(odometryPose);
      double distToB =
          field.getTagPose(12).get().toPose2d().getTranslation().getDistance(odometryPose);

      if (distToT > distToB + hpThreshold) {
        return FieldElement.HB;
      } else if (distToB > distToT + hpThreshold) {
        return FieldElement.HT;
      } else {
        ChassisSpeeds fieldRelative =
            ChassisSpeeds.fromRobotRelativeSpeeds(
                m_Swerve.getChassisSpeeds(), m_Swerve.getRotation2D());
        return fieldRelative.vyMetersPerSecond > 0 ? FieldElement.HT : FieldElement.HB;
      }
    } else {
      // RED:
      double distToT =
          field.getTagPose(1).get().toPose2d().getTranslation().getDistance(odometryPose);
      double distToB =
          field.getTagPose(2).get().toPose2d().getTranslation().getDistance(odometryPose);

      if (distToT > distToB + hpThreshold) {
        return FieldElement.HB;
      } else if (distToB > distToT + hpThreshold) {
        return FieldElement.HT;
      } else {
        ChassisSpeeds fieldRelative =
            ChassisSpeeds.fromRobotRelativeSpeeds(
                m_Swerve.getChassisSpeeds(), m_Swerve.getRotation2D());
        return fieldRelative.vyMetersPerSecond < 0 ? FieldElement.HT : FieldElement.HB;
      }
    }
  }
}
