// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Drivetrain;

import static edu.wpi.first.units.Units.*;
import static frc.robot.Constants.DrivetrainConstants.ControllerConstants.*;
import static frc.robot.Constants.DrivetrainConstants.ControllerConstants.OdometryAlign.*;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.lib.dashboard.TunableNumber;

/** Add your docs here. */
public class PoseAlignController {

  private ProfiledPIDController xController;
  private ProfiledPIDController yController;
  private RotationController thetaController;
  // private ProfiledPIDController thetaController;

  private final Swerve m_Swerve;

  private TunableNumber[] xPID = {
    new TunableNumber("AutoAlign/Px", xP),
    new TunableNumber("AutoAlign/Ix", xI),
    new TunableNumber("AutoAlign/Dx", xD)
  };

  private TunableNumber[] yPID = {
    new TunableNumber("AutoAlign/Py", yP),
    new TunableNumber("AutoAlign/Iy", yI),
    new TunableNumber("AutoAlign/Dy", yD)
  };

  private TunableNumber[] thetaPID = {
    new TunableNumber("AutoAlign/Pt", tP),
    new TunableNumber("AutoAlign/It", tI),
    new TunableNumber("AutoAlign/Dt", tD)
  };

  public PoseAlignController(Swerve swerve) {
    xController = new ProfiledPIDController(0, 0, 0, new TrapezoidProfile.Constraints(4, 4));
    yController = new ProfiledPIDController(0, 0, 0, new TrapezoidProfile.Constraints(4, 4));
    thetaController = new RotationController(swerve);
    // thetaController = new ProfiledPIDController(0, 0, 0, new TrapezoidProfile.Constraints(0, 0));

    // thetaController.enableContinuousInput(-Math.PI, Math.PI);

    xController.setTolerance(toleranceTranslation);
    yController.setTolerance(toleranceTranslation);
    // thetaController.setTolerance(toleranceRadians);

    xController.setConstraints(
        new TrapezoidProfile.Constraints(
            maxVelocity.in(MetersPerSecond) / Math.sqrt(2),
            maxAcceleration.in(MetersPerSecondPerSecond) / Math.sqrt(2)));
    yController.setConstraints(
        new TrapezoidProfile.Constraints(
            maxVelocity.in(MetersPerSecond) / Math.sqrt(2),
            maxAcceleration.in(MetersPerSecondPerSecond) / Math.sqrt(2)));
    // thetaController.setConstraints(
    //     new TrapezoidProfile.Constraints(
    //         maxAngularVelocityAuto.in(RadiansPerSecond),
    //         maxAngularAccelerationAuto.in(RadiansPerSecondPerSecond)));

    m_Swerve = swerve;
    reset();
  }

  public void reset() {
    xController.reset(m_Swerve.getPose().getX(), m_Swerve.getChassisSpeeds().vxMetersPerSecond);
    yController.reset(m_Swerve.getPose().getY(), m_Swerve.getChassisSpeeds().vyMetersPerSecond);
    // thetaController.reset(
    //     m_Swerve.getRotation2D().getRadians(),
    // m_Swerve.getChassisSpeeds().omegaRadiansPerSecond);
  }

  public ChassisSpeeds update(Pose2d currentPose, Pose2d targetPose) {
    /* Update PID Controllers */
    xController.setPID(xPID[0].getNumber(), xPID[1].getNumber(), xPID[2].getNumber());
    yController.setPID(yPID[0].getNumber(), yPID[1].getNumber(), yPID[2].getNumber());
    // thetaController.setPID(
    //     thetaPID[0].getNumber(), thetaPID[1].getNumber(), thetaPID[2].getNumber());

    double xVel = xController.calculate(currentPose.getX(), targetPose.getX());
    double yVel = yController.calculate(currentPose.getY(), targetPose.getY());

    double radiansSetpoint = targetPose.getRotation().getRadians();

    // double thetaVel =
    thetaController.update(
        Rotation2d.fromRadians(radiansSetpoint), thetaPID[0].getNumber(), thetaPID[2].getNumber());
    // thetaController.calculate(m_Swerve.getRotation2D().getRadians(), radiansSetpoint);

    return ChassisSpeeds.fromFieldRelativeSpeeds(
        xVel, yVel, thetaController.getOutput(), currentPose.getRotation());
  }

  public boolean atGoal() {
    return xController.atGoal() && yController.atGoal() && thetaController.atGoal();
  }
}
