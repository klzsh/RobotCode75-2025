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
import edu.wpi.first.wpilibj.Timer;
import frc.lib.dashboard.TunableNumber;
import frc.robot.subsystems.Vision.AprilTagCamera;

/** Add your docs here. */
public class VisionController {

  private ProfiledPIDController xController;
  private ProfiledPIDController yController;
  private ProfiledPIDController thetaController;

  private final Swerve m_Swerve;

  private double lastSeenAprilTagTime;

  private Pose2d currentPose;
  private Pose2d targetPose;

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

  private TunableNumber[] thetaPID = {
    new TunableNumber("VisionController/Pt", tP),
    new TunableNumber("VisionController/It", tI),
    new TunableNumber("VisionController/Dt", tD)
  };

  public VisionController(Swerve swerve) {
    xController = new ProfiledPIDController(0, 0, 0, new TrapezoidProfile.Constraints(0, 0));
    yController = new ProfiledPIDController(0, 0, 0, new TrapezoidProfile.Constraints(0, 0));
    thetaController = new ProfiledPIDController(0, 0, 0, new TrapezoidProfile.Constraints(0, 0));

    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    xController.setTolerance(toleranceTranslation);
    yController.setTolerance(toleranceTranslation);
    thetaController.setTolerance(toleranceRadians);

    xController.setConstraints(
        new TrapezoidProfile.Constraints(
            maxVelocity.in(MetersPerSecond) / Math.sqrt(2),
            maxAcceleration.in(MetersPerSecondPerSecond) / Math.sqrt(2)));
    yController.setConstraints(
        new TrapezoidProfile.Constraints(
            maxVelocity.in(MetersPerSecond) / Math.sqrt(2),
            maxAcceleration.in(MetersPerSecondPerSecond) / Math.sqrt(2)));
    thetaController.setConstraints(
        new TrapezoidProfile.Constraints(
            maxAngularVelocity.in(RadiansPerSecond),
            maxAngularAcceleration.in(RadiansPerSecondPerSecond)));

    currentPose = new Pose2d();

    m_Swerve = swerve;
    reset();
  }

  public void reset() {
    xController.reset(m_Swerve.getPose().getX(), m_Swerve.getChassisSpeeds().vxMetersPerSecond);
    yController.reset(m_Swerve.getPose().getY(), m_Swerve.getChassisSpeeds().vyMetersPerSecond);
    thetaController.reset(
        m_Swerve.getRotation2D().getRadians(), m_Swerve.getChassisSpeeds().omegaRadiansPerSecond);
  }

  public ChassisSpeeds update(AprilTagCamera primaryCamera, int targetTagID) {
    /* Update PID Controllers */
    xController.setPID(xPID[0].getNumber(), xPID[1].getNumber(), xPID[2].getNumber());
    yController.setPID(yPID[0].getNumber(), yPID[1].getNumber(), yPID[2].getNumber());
    thetaController.setPID(
        thetaPID[0].getNumber(), thetaPID[1].getNumber(), thetaPID[2].getNumber());

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
      double robotToAprilTagAngle = Math.PI / 2 - absoluteAngleToTag;

      xDisplacement = targetMeters * Math.cos(absoluteAngleToTag);
      yDisplacement = targetMeters * Math.sin(absoluteAngleToTag);

      lastSeenAprilTagTime = Timer.getFPGATimestamp();
    }

    double xVel = xController.calculate(xDisplacement, 0);
    double yVel = yController.calculate(yDisplacement, 0);

    // double radiansSetpoint = fieldPoses.get(targetPose).getRotation().getRadians();

    double thetaVel =
        -thetaController.calculate(m_Swerve.getRotation2D().getRadians(), 0);

    return ChassisSpeeds.fromFieldRelativeSpeeds(xVel, yVel, thetaVel, currentPose.getRotation());
  }

  public boolean atGoal() {
    return xController.atGoal() && yController.atGoal() && thetaController.atGoal();
  }
}
