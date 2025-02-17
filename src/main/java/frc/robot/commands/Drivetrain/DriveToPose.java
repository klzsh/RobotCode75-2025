// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drivetrain;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain.AutoAlignController;
import frc.robot.subsystems.Drivetrain.Swerve;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class DriveToPose extends Command {
  private Swerve m_swerve;
  private AutoAlignController m_controller;
  private Pose2d targetPose;
  private boolean holdPose;

  /** Creates a new DriveToPose. */
  public DriveToPose(Swerve swerve, Pose2d pose, boolean hold) {
    m_swerve = swerve;
    targetPose = pose;
    holdPose = hold;
    m_controller = new AutoAlignController(m_swerve);
    // Use addRequirements() here to declare subsystem dependencies.
    // addRequirements(m_swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_controller.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    ChassisSpeeds speeds = m_controller.update(m_swerve.getPose(), targetPose);
    m_swerve.setChassisSpeeds(speeds);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return !holdPose && m_controller.atGoal();
  }
}
