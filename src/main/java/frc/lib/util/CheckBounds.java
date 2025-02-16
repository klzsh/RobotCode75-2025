package frc.lib.util;

import java.util.Map;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.lib.util.FieldPose.FieldElement;
import frc.lib.util.FieldPose.Offset;

public class CheckBounds {
    public static class Bound {
        public Translation2d point1;
        public Translation2d point2;
        public FieldPose.FieldElement element;
        public Alliance alliance;

        public boolean isWithin(Translation2d pose) {
            double minX = Math.min(point1.getX(), point2.getX());
            double maxX = Math.max(point1.getX(), point2.getX());

            double minY = Math.min(point1.getY(), point2.getY());
            double maxY = Math.max(point1.getY(), point2.getY());
            
            return (pose.getX() < maxX && pose.getX() > minX && pose.getY() < maxY && pose.getY() > minY);
        }

        public Bound(double x1, double y1, double x2, double y2, FieldPose.FieldElement element, Alliance alliance) {
            point1 = new Translation2d(x1, y1);
            point2 = new Translation2d(x2, y2);
            this.element = element;
            this.alliance = alliance;
        }

        public Bound(Translation2d p1, Translation2d p2, FieldPose.FieldElement element, Alliance alliance) {
            point1 = p1;
            point2 = p2;
            this.element = element;
            this.alliance = alliance;

        }
    }
    
     public static Bound[] bounds = {
        new Bound(0, 0, 0, 0, FieldElement.RBL, Alliance.Blue),
        new Bound(0, 0, 0, 0, FieldElement.RL, Alliance.Blue),
        new Bound(0, 0, 0, 0, FieldElement.RTL, Alliance.Blue),
        new Bound(0, 0, 0, 0, FieldElement.RBR, Alliance.Blue),
        new Bound(0, 0, 0, 0, FieldElement.RR, Alliance.Blue),
        new Bound(0, 0, 0, 0, FieldElement.RTR, Alliance.Blue),
        new Bound(0, 0, 0, 0, FieldElement.RBL, Alliance.Red),
        new Bound(0, 0, 0, 0, FieldElement.RL, Alliance.Red),
        new Bound(0, 0, 0, 0, FieldElement.RTL, Alliance.Red),
        new Bound(0, 0, 0, 0, FieldElement.RBR, Alliance.Red),
        new Bound(0, 0, 0, 0, FieldElement.RR, Alliance.Red),
        new Bound(0, 0, 0, 0, FieldElement.RTR, Alliance.Red),
     };
}