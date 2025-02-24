// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Drivetrain;

import static edu.wpi.first.units.Units.*;
import static frc.robot.Constants.DrivetrainConstants.ControllerConstants.*;
import static frc.robot.Constants.DrivetrainConstants.ControllerConstants.OdometryAlign.*;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Importance;
import edu.wpi.first.epilogue.Logged.Strategy;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.lib.dashboard.TunableNumber;

/** Add your docs here. */
@Logged(name = "Pose Controller", strategy = Strategy.OPT_IN)
public class PoseAlignController {

  private ProfiledPIDController translationController;
  // already logged
  private RotationController thetaController;
  @Logged(importance = Importance.INFO)
  private Pose2d targetPose;

  // private ProfiledPIDController thetaController;
  //@Logged(importance = Importance.INFO)
  private ChassisSpeeds output;

  private final Swerve m_Swerve;

  private TunableNumber[] translationPID = {
    new TunableNumber("AutoAlign/Ptr", xP),
    new TunableNumber("AutoAlign/Itr", xI),
    new TunableNumber("AutoAlign/Dtr", xD)
  };

  private TunableNumber[] thetaPID = {
    new TunableNumber("AutoAlign/Pt", tP),
    new TunableNumber("AutoAlign/It", tI),
    new TunableNumber("AutoAlign/Dt", tD)
  };

  public PoseAlignController(Swerve swerve) {
    translationController =
        new ProfiledPIDController(0, 0, 0, new TrapezoidProfile.Constraints(4, 4));
    thetaController = new RotationController(swerve);
    output = new ChassisSpeeds();

    translationController.setTolerance(toleranceTranslation);
    // thetaController.setTolerance(toleranceRadians);

    translationController.setConstraints(
        new TrapezoidProfile.Constraints(
            maxVelocity.in(MetersPerSecond), maxAcceleration.in(MetersPerSecondPerSecond)));
    m_Swerve = swerve;
    reset();
  }

  public double getVelocityFromSwerve() {
    return Math.sqrt(
        Math.pow(m_Swerve.getChassisSpeeds().vxMetersPerSecond, 2)
            + Math.pow(m_Swerve.getChassisSpeeds().vyMetersPerSecond, 2));
  }

  public void reset() {
    translationController.reset(
        0, getVelocityFromSwerve()); // TODO: might need to put an actual value for the position
  }

  public ChassisSpeeds update(Pose2d currentPose, Pose2d targetPose) {
    this.targetPose = targetPose;
    /* Update PID Controllers */
    translationController.setPID(
        translationPID[0].getNumber(),
        translationPID[1].getNumber(),
        translationPID[2].getNumber());
    // thetaController.setPID(
    //     thetaPID[0].getNumber(), thetaPID[1].getNumber(), thetaPID[2].getNumber());

    double angleToTarget =
        Math.atan2(targetPose.getY() - currentPose.getY(), targetPose.getX() - currentPose.getX());
    double distanceToTarget =
        Math.hypot(targetPose.getY() - currentPose.getY(), targetPose.getX() - currentPose.getX());

    double velocity = translationController.calculate(distanceToTarget, 0);
    double xVel = -velocity * Math.cos(angleToTarget);
    double yVel = -velocity * Math.sin(angleToTarget);

    double radiansSetpoint = targetPose.getRotation().getRadians();
    thetaController.update(
        Rotation2d.fromRadians(radiansSetpoint), thetaPID[0].getNumber(), thetaPID[2].getNumber());
    // thetaController.calculate(m_Swerve.getRotation2D().getRadians(), radiansSetpoint);

    output =
        ChassisSpeeds.fromFieldRelativeSpeeds(
            xVel, yVel, thetaController.getOutput(), currentPose.getRotation());

    return ChassisSpeeds.fromFieldRelativeSpeeds(
        xVel, yVel, thetaController.getOutput(), currentPose.getRotation());
  }

  //@Logged(importance = Importance.INFO)
  public boolean atGoal() {
    return translationController.atGoal() && thetaController.atGoal();
  }
}
