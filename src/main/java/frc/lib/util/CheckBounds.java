package frc.lib.util;

import java.util.Map;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;

public class CheckBounds {
    public class Bound {
        public Translation2d point1;
        public Translation2d point2;

        public boolean isWithin(Translation2d pose) {
            double minX = Math.min(point1.getX(), point2.getX());
            double maxX = Math.max(point1.getX(), point2.getX());

            double minY = Math.min(point1.getY(), point2.getY());
            double maxY = Math.max(point1.getY(), point2.getY());
            
            return (pose.getX() < maxX && pose.getX() > minX && pose.getY() < maxY && pose.getY() > minY);
        }

        public Bound(double x1, double y1, double x2, double y2) {
            point1 = new Translation2d(x1, y1);
            point2 = new Translation2d(x2, y2);
        }

        public Bound(Translation2d p1, Translation2d p2) {
            point1 = p1;
            point2 = p2;
        }
    }
    
     public static final Map<FieldPose, Bound> fieldPoses =
      Map.ofEntries(
          // processor
          entry(
              new FieldPose(Alliance.Blue, FieldElement.P, Offset.MID),
              new Pose2d(0.0, 0.0, new Rotation2d(0.0))
          ));
}