// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.subsystems.Drivetrain;

// import static edu.wpi.first.units.Units.*;

// import edu.wpi.first.math.controller.ProfiledPIDController;
// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Translation2d;
// import edu.wpi.first.math.kinematics.ChassisSpeeds;
// import edu.wpi.first.math.trajectory.TrapezoidProfile;
// import edu.wpi.first.wpilibj.Timer;
// import frc.lib.dashboard.TunableNumber;
// import frc.robot.Constants.DrivetrainConstants;
// import frc.robot.Constants.VisionConstants;
// import frc.robot.subsystems.Vision.AprilTagCamera;

// import static frc.robot.Constants.VisionConstants.*;

// /** Add your docs here. */
// public class VisionController {

//     public enum AlignTargets {
//         REEF_LEFT(REEF_LEFT_OFFSET),
//         REEF_RIGHT(REEF_RIGHT_OFFSET),
//         HUMAN_PLAYER_LEFT(HP_LEFT_OFFSET),
//         HUMAN_PLAYER_CENTER(HP_CENTER_OFFSET),
//         HUMAN_PLAYER_RIGHT(HP_RIGHT_OFFSET);
//         public final Translation2d frontLeftOffset;
//         public final Translation2d frontCenterOffset;
//         public final Translation2d backOffset;

//         AlignTargets(Translation2d[] offset) {
//             this.frontLeftOffset = offset[0];
//             this.frontCenterOffset = offset[1];
//             this.backOffset = offset[2];
//         }
//     }

//     private ProfiledPIDController xController;
//     private ProfiledPIDController yController;
//     private ProfiledPIDController thetaController;

//     private final Swerve m_Swerve;

//     private AprilTagCamera FrontLeftCamera; // coral side, 0
//     private AprilTagCamera FrontCenterCamera; // algae, 1
//     private AprilTagCamera BackCamera; // human player station, 2

//     private double lastSeenAprilTagTime;
//     private AutoAlignController fallbackController;

//     private TunableNumber[] xPID = {
//             new TunableNumber("VisionController/Px", 0),
//             new TunableNumber("VisionController/Ix", 0),
//             new TunableNumber("VisionController/Dx", 0)
//     };

//     private TunableNumber[] yPID = {
//             new TunableNumber("VisionController/Py", 0),
//             new TunableNumber("VisionController/Iy", 0),
//             new TunableNumber("VisionController/Dy", 0)
//     };

//     private TunableNumber[] thetaPID = {
//             new TunableNumber("VisionController/Pt", 0),
//             new TunableNumber("VisionController/It", 0),
//             new TunableNumber("VisionController/Dt", 0)
//     };

//     public VisionController(Swerve swerve, AprilTagCamera frontLeftCamera, AprilTagCamera frontCenterCamera, AprilTagCamera backCamera, AutoAlignController fallback) {
//         xController = new ProfiledPIDController(0, 0, 0, new TrapezoidProfile.Constraints(0, 0));
//         yController = new ProfiledPIDController(0, 0, 0, new TrapezoidProfile.Constraints(0, 0));
//         thetaController = new ProfiledPIDController(0, 0, 0, new TrapezoidProfile.Constraints(0, 0));

//         thetaController.enableContinuousInput(-Math.PI, Math.PI);

//         xController.setTolerance(DrivetrainConstants.ControllerConstants.toleranceTranslation);
//         yController.setTolerance(DrivetrainConstants.ControllerConstants.toleranceTranslation);
//         thetaController.setTolerance(DrivetrainConstants.ControllerConstants.toleranceRadians);

//         xController.setConstraints(
//                 new TrapezoidProfile.Constraints(
//                         DrivetrainConstants.ControllerConstants.maxVelocity.in(MetersPerSecond) / Math.sqrt(2),
//                         DrivetrainConstants.ControllerConstants.maxAcceleration.in(MetersPerSecondPerSecond)
//                                 / Math.sqrt(2)));
//         yController.setConstraints(
//                 new TrapezoidProfile.Constraints(
//                         DrivetrainConstants.ControllerConstants.maxVelocity.in(MetersPerSecond) / Math.sqrt(2),
//                         DrivetrainConstants.ControllerConstants.maxAcceleration.in(MetersPerSecondPerSecond)
//                                 / Math.sqrt(2)));
//         thetaController.setConstraints(
//                 new TrapezoidProfile.Constraints(
//                         DrivetrainConstants.ControllerConstants.maxAngularVelocity.in(RadiansPerSecond),
//                         DrivetrainConstants.ControllerConstants.maxAngularAcceleration.in(
//                                 RadiansPerSecondPerSecond)));

//         FrontLeftCamera = frontLeftCamera;
//         FrontCenterCamera = frontCenterCamera;
//         BackCamera = backCamera;
//         fallbackController = fallback;

//         m_Swerve = swerve;
//         reset();
//     }

//     public void reset() {
//         xController.reset(m_Swerve.getPose().getX(), m_Swerve.getChassisSpeeds().vxMetersPerSecond);
//         yController.reset(m_Swerve.getPose().getY(), m_Swerve.getChassisSpeeds().vyMetersPerSecond);
//         thetaController.reset(
//                 m_Swerve.getRotation2D().getRadians(), m_Swerve.getChassisSpeeds().omegaRadiansPerSecond);
//     }

//     public ChassisSpeeds update(Pose2d currentPose, int targetTagID, AlignTargets alignTarget) {
//         /* Update PID Controllers */
//         xController.setPID(xPID[0].getNumber(), xPID[1].getNumber(), xPID[2].getNumber());
//         yController.setPID(yPID[0].getNumber(), yPID[1].getNumber(), yPID[2].getNumber());
//         thetaController.setPID(
//                 thetaPID[0].getNumber(), thetaPID[1].getNumber(), thetaPID[2].getNumber());

//         double currentX = 0;
//         double currentY = 0;
//         double targetX = 0;
//         double targetY = 0;
//         boolean hasTarget = true;

//         switch (alignTarget) {
//             case REEF_LEFT -> {
//                 if (FrontCenterCamera.getTarget(targetTagID).isEmpty()) {
//                     if (FrontLeftCamera.getTarget(targetTagID).isEmpty()) {
//                         hasTarget = false;
//                     } else {
//                         targetX = alignTarget.frontLeftOffset.getX();
//                         targetY = alignTarget.frontLeftOffset.getY();
//                         currentX = FrontLeftCamera.getX(targetTagID).getAsDouble();
//                         currentY = FrontLeftCamera.getY(targetTagID).getAsDouble();
//                     }
//                 } else {
//                     targetX = alignTarget.frontCenterOffset.getX();
//                     targetY = alignTarget.frontCenterOffset.getY();
//                     currentX = FrontCenterCamera.getX(targetTagID).getAsDouble();
//                     currentY = FrontCenterCamera.getY(targetTagID).getAsDouble();
//                 }
//             }
//             case REEF_RIGHT -> {
//                 if (FrontLeftCamera.getTarget(targetTagID).isEmpty()) {
//                     if (FrontCenterCamera.getTarget(targetTagID).isEmpty()) {
//                         hasTarget = false;
//                     } else {
//                         targetX = alignTarget.frontCenterOffset.getX();
//                         targetY = alignTarget.frontCenterOffset.getY();
//                         currentX = FrontCenterCamera.getX(targetTagID).getAsDouble();
//                         currentY = FrontCenterCamera.getY(targetTagID).getAsDouble();
//                     }
//                 } else {
//                     targetX = alignTarget.frontLeftOffset.getX();
//                     targetY = alignTarget.frontLeftOffset.getY();
//                     currentX = FrontLeftCamera.getX(targetTagID).getAsDouble();
//                     currentY = FrontLeftCamera.getY(targetTagID).getAsDouble();
//                 }
//             }
//             case HUMAN_PLAYER -> {
//                 if (BackCamera.getTarget(targetTagID).isEmpty()) {
//                     hasTarget = false;
//                 } else {
//                     targetX = alignTarget.backOffset.getX();
//                     targetY = alignTarget.backOffset.getY();
//                     currentX = BackCamera.getX(targetTagID).getAsDouble();
//                     currentY = BackCamera.getY(targetTagID).getAsDouble();
//                 }
//             }
//         }

//         if (hasTarget) {
//             lastSeenAprilTagTime = Timer.getFPGATimestamp();
//         } else {
//             if ((Timer.getFPGATimestamp() - lastSeenAprilTagTime) > maxTimeUntilFallbackToOdometry) {
//                 return fallbackController.update(m_Swerve.getPose(), aprilTagTargets.get(targetTagID));
//             } else {
//                 return m_Swerve.getChassisSpeeds();
//             }
//         }

//         double xVel = xController.calculate(currentX, targetX);
//         double yVel = yController.calculate(currentY, targetY);

//         double radiansSetpoint = aprilTagTargets.get(targetTagID).getRotation().getRadians();

//         double thetaVel =
//                 -thetaController.calculate(m_Swerve.getRotation2D().getRadians(), radiansSetpoint);

//         return ChassisSpeeds.fromFieldRelativeSpeeds(xVel, yVel, thetaVel, currentPose.getRotation());
//     }

//     public boolean atGoal() {
//         return xController.atGoal() && yController.atGoal() && thetaController.atGoal();
//     }
// }
