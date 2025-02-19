// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Drivetrain;

import static edu.wpi.first.units.Units.*;
import static frc.robot.Constants.DrivetrainConstants.ControllerConstants.*;
import static frc.robot.Constants.DrivetrainConstants.ControllerConstants.VisionAlign.*;
import static frc.robot.Constants.FieldConstants.*;
import static frc.robot.Constants.VisionConstants.*;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.lib.dashboard.TunableNumber;
import frc.robot.subsystems.Vision.AprilTagCamera;

/** Add your docs here. */
public class VisionTranslationController {

  private ProfiledPIDController xController;
  private ProfiledPIDController yController;

  private final Swerve m_Swerve;

  private Pose2d currentPose;

  private TunableNumber[] xPID = {
    new TunableNumber("VisionController/Px", xP),
    new TunableNumber("VisionController/Ix", xI),
    new TunableNumber("VisionController/Dx", xD)
  };

  private TunableNumber[] yPID = {
    new TunableNumber("VisionController/Py", yP),
    new TunableNumber("VisionController/Iy", yI),
    new TunableNumber("VisionController/Dy", yD)
  };

  public VisionTranslationController(Swerve swerve) {
    xController = new ProfiledPIDController(0, 0, 0, new TrapezoidProfile.Constraints(0, 0));
    yController = new ProfiledPIDController(0, 0, 0, new TrapezoidProfile.Constraints(0, 0));

    xController.setTolerance(toleranceTranslation);
    yController.setTolerance(toleranceTranslation);

    xController.setConstraints(
        new TrapezoidProfile.Constraints(
            maxVelocity.in(MetersPerSecond) / Math.sqrt(2),
            maxAcceleration.in(MetersPerSecondPerSecond) / Math.sqrt(2)));
    yController.setConstraints(
        new TrapezoidProfile.Constraints(
            maxVelocity.in(MetersPerSecond) / Math.sqrt(2),
            maxAcceleration.in(MetersPerSecondPerSecond) / Math.sqrt(2)));

    currentPose = new Pose2d();

    m_Swerve = swerve;
    reset();
  }

  public void reset() {
    xController.reset(m_Swerve.getPose().getX(), m_Swerve.getChassisSpeeds().vxMetersPerSecond);
    yController.reset(m_Swerve.getPose().getY(), m_Swerve.getChassisSpeeds().vyMetersPerSecond);
  }

  public ChassisSpeeds update(AprilTagCamera primaryCamera, int targetTagID) {
    /* Update PID Controllers */
    xController.setPID(xPID[0].getNumber(), xPID[1].getNumber(), xPID[2].getNumber());
    yController.setPID(yPID[0].getNumber(), yPID[1].getNumber(), yPID[2].getNumber());

    currentPose = m_Swerve.getPose();

    double tX = 0;
    boolean hasTarget = true;
    double targetYaw = 0;
    double targetMeters = 0;
    double xDisplacement = 0;
    double yDisplacement = 0;

    if (primaryCamera.getTarget(targetTagID).isPresent()) {
      tX = primaryCamera.getX(targetTagID).getAsDouble();

      double thetaCalc = Math.asin(tX / targetMeters);
      double absoluteAngleToTag = thetaCalc + m_Swerve.getRotation2D().getRadians();

      xDisplacement = targetMeters * Math.cos(absoluteAngleToTag);
      yDisplacement = targetMeters * Math.sin(absoluteAngleToTag);
    }
    double xVel = xController.calculate(xDisplacement, 0);
    double yVel = yController.calculate(yDisplacement, 0);

    return ChassisSpeeds.fromFieldRelativeSpeeds(xVel, yVel, 0, currentPose.getRotation());
  }

  public boolean atGoal() {
    return xController.atGoal() && yController.atGoal();
  }
}
