// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drivetrain;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.util.CheckBounds;
import frc.lib.util.FieldPose;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.subsystems.Drivetrain.Swerve;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ChezyPose extends Command {
  private Swerve m_swerve;
  private FieldPose targetPose = null;
  private Pose2d targetPose2d = null;
  private boolean holdPose;
  private final ProfiledPIDController driveController =
      new ProfiledPIDController(
          DrivetrainConstants.ControllerConstants.OdometryAlign.xP,
          0.0,
          0.0,
          new TrapezoidProfile.Constraints(0.0, 0.0),
          0.02);
  private final ProfiledPIDController thetaController =
      new ProfiledPIDController(
          DrivetrainConstants.ControllerConstants.OdometryAlign.tP,
          0.0,
          0.0,
          new TrapezoidProfile.Constraints(0.0, 0.0),
          0.02);
  private Translation2d lastSetpointTranslation;
  private double driveErrorAbs;
  private double thetaErrorAbs;
  private double ffMinRadius = 0.2, ffMaxRadius = 0.8;

  /** Creates a new DriveToPose. */
  public ChezyPose(Swerve swerve, FieldPose pose, boolean hold) {
    m_swerve = swerve;
    targetPose = pose;
    holdPose = hold;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_swerve);
  }

  public ChezyPose(Swerve swerve, Pose2d pose, boolean hold) {
    m_swerve = swerve;
    targetPose2d = pose;
    holdPose = hold;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (targetPose2d == null) {
      targetPose2d = CheckBounds.getNearestFieldPose2d(m_swerve, targetPose);
    }
    Pose2d currentPose = m_swerve.getPose();
    driveController.reset(
        currentPose.getTranslation().getDistance(targetPose2d.getTranslation()),
        Math.min(
            0.0,
            -new Translation2d(
                    m_swerve.getChassisSpeeds().vxMetersPerSecond,
                    m_swerve.getChassisSpeeds().vyMetersPerSecond)
                .rotateBy(
                    targetPose2d
                        .getTranslation()
                        .minus(currentPose.getTranslation())
                        .getAngle()
                        .unaryMinus())
                .getX()));
    thetaController.reset(
        currentPose.getRotation().getRadians(), m_swerve.getChassisSpeeds().omegaRadiansPerSecond);
    lastSetpointTranslation = currentPose.getTranslation();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Pose2d currentPose = m_swerve.getPose();

    double currentDistance =
        currentPose.getTranslation().getDistance(targetPose2d.getTranslation());
    double ffScaler =
        MathUtil.clamp((currentDistance - ffMinRadius) / (ffMaxRadius - ffMinRadius), 0.0, 0.5);
    driveErrorAbs = currentDistance;
    driveController.reset(
        lastSetpointTranslation.getDistance(targetPose2d.getTranslation()),
        driveController.getSetpoint().velocity);
    double driveVelocityScalar =
        driveController.getSetpoint().velocity * ffScaler
            + driveController.calculate(driveErrorAbs, 0.0);
    if (currentDistance < driveController.getPositionTolerance()) driveVelocityScalar = 0.0;
    lastSetpointTranslation =
        new Pose2d(
                targetPose2d.getTranslation(),
                currentPose.getTranslation().minus(targetPose2d.getTranslation()).getAngle())
            .transformBy(
                new Transform2d(
                    new Translation2d(driveController.getSetpoint().position, 0.0),
                    new Rotation2d()))
            .getTranslation();

    // Calculate theta speed
    double thetaVelocity =
        thetaController.getSetpoint().velocity * ffScaler * 0.7
            + thetaController.calculate(
                currentPose.getRotation().getRadians(), targetPose2d.getRotation().getRadians());
    thetaErrorAbs =
        Math.abs(currentPose.getRotation().minus(targetPose2d.getRotation()).getRadians());
    if (thetaErrorAbs < thetaController.getPositionTolerance()) thetaVelocity = 0.0;

    // Command speeds
    Translation2d driveVelocity =
        new Pose2d(
                0, 0, currentPose.getTranslation().minus(targetPose2d.getTranslation()).getAngle())
            .transformBy(
                new Transform2d(new Translation2d(driveVelocityScalar, 0.0), new Rotation2d()))
            .getTranslation();
    m_swerve.setChassisSpeeds(
        new ChassisSpeeds(driveVelocity.getX(), driveVelocity.getY(), thetaVelocity));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_swerve.setChassisSpeeds(new ChassisSpeeds(0, 0, 0));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (driveController.atGoal() && thetaController.atGoal());
  }
}
