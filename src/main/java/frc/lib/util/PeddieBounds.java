package frc.lib.util;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.FieldConstants;
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

public class PeddieBounds {

  private static Swerve m_Swerve;
  private static List<Translation2d> badHexagonPoints;

  private static double cosineSimilarity(Translation2d a, Translation2d b) {
    return (a.getX() * b.getX() + a.getY() * b.getY()) / (a.getNorm() * b.getNorm());
  }

  public static void init(Swerve swerve) {
    m_Swerve = swerve;
    Translation2d reefCenter;
    // BLUE
    if (DriverStation.getAlliance().isEmpty()
        || DriverStation.getAlliance().get() == DriverStation.Alliance.Blue) {
      Pose2d tag18 =
          AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded)
              .getTagPose(18)
              .get()
              .toPose2d();
      Pose2d tag21 =
          AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded)
              .getTagPose(21)
              .get()
              .toPose2d();
      reefCenter =
          new Translation2d((tag18.getX() + tag21.getX()) / 2, (tag18.getY() + tag21.getY()) / 2);
    }
    // RED
    else {
      Pose2d tag7 =
          AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded)
              .getTagPose(7)
              .get()
              .toPose2d();
      Pose2d tag10 =
          AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded)
              .getTagPose(10)
              .get()
              .toPose2d();
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
    // TODO: tune
    reefCornerToCenter += 1.2;

    badHexagonPoints = new ArrayList<>();
    for (int i = 0; i < 360; i += 60) {
      badHexagonPoints.add(
          new Translation2d(
              reefCenter.getX() + reefCornerToCenter * Math.cos(Math.toRadians(i)),
              reefCenter.getY() + reefCornerToCenter * Math.sin(Math.toRadians(i))));
    }

    Field2d[] fields = new Field2d[6];
    for (int i = 0; i < 6; i++) {
      Translation2d p = badHexagonPoints.get(i);
      fields[i] = new Field2d();
      fields[i].setRobotPose(new Pose2d(p.getX(), p.getY(), new Rotation2d(0)));
      SmartDashboard.putData("hexagon " + i, fields[i]);
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

    return !(Math.abs(angle) < Math.PI);
  }

  public static boolean insideBadHexagon(Pose2d p) {
    return insideBadHexagon(new Translation2d(p.getX(), p.getY()));
  }

  public static int getReefID() {
    // calculate current odometry pose
    Translation2d odometryPose = m_Swerve.getPose().getTranslation();
    List<IDVectorPair> robotToTag = new ArrayList<>();

    // BLUE:
    if (DriverStation.getAlliance().isEmpty()
        || DriverStation.getAlliance().get() == DriverStation.Alliance.Blue) {
      for (int i = 17; i <= 22; i++) {
        Translation2d tagPose =
            AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded)
                .getTagPose(i)
                .get()
                .toPose2d()
                .getTranslation();
        robotToTag.add(new IDVectorPair(i, tagPose.minus(odometryPose)));
      }
    }
    // RED:
    else {
      for (int i = 6; i <= 11; i++) {
        Translation2d tagPose =
            AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded)
                .getTagPose(i)
                .get()
                .toPose2d()
                .getTranslation();
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

    if (isInBadHexagon) return tag0gyroError < 30.0 ? tag0id : 0;

    if (robotToTag.get(1).vector.getNorm() - robotToTag.get(0).vector.getNorm() >= 0.25)
      return tag0gyroError < 45.0 ? tag0id : 0;

    ChassisSpeeds speeds = m_Swerve.getChassisSpeeds();
    Translation2d robotMovement =
        new Translation2d(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);

    int bestTag;
    if (robotMovement.getNorm() == 0) bestTag = tag0id;
    else {
      double similar0 = cosineSimilarity(robotToTag.get(0).vector, robotMovement);
      double similar1 = cosineSimilarity(robotToTag.get(1).vector, robotMovement);
      SmartDashboard.putNumber("tag 0 similarity", similar0);
      SmartDashboard.putNumber("tag 1 similarity", similar1);
      bestTag = similar0 >= similar1 ? tag0id : tag1id;
    }

    double bestTagNeededAngle = FieldConstants.tagToHeadingMap.get(bestTag);
    return Math.abs(m_Swerve.getRotationDegrees() - bestTagNeededAngle) < 45.0 ? bestTag : 0;
  }
}