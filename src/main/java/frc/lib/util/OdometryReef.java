package frc.lib.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;

public class OdometryReef {
    public class bounds {
        public Translation2d point1;
        public Translation2d point2;

        public boolean isWithin(Translation2d pose) {
            double minX = Math.min(point1.getX(), point2.getX())
            double maxY = Math.max(point1.getX(), point2.getX())
            
            if (pose.getX() < point1.getX() && pose.getX() > point2.getX()) {
                
            }
        }
    } 
}