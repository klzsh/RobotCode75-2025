// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drivetrain;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.util.CheckBounds;
import frc.lib.util.FieldPose;
import frc.robot.subsystems.Drivetrain.PoseAlignController;
import frc.robot.subsystems.Drivetrain.Swerve;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class DriveToPose extends Command {
  private Swerve m_swerve;
  private PoseAlignController m_controller;
  private FieldPose targetPose = null;
  private Pose2d targetPose2d = null;
  private boolean holdPose;

  /** Creates a new DriveToPose. */
  public DriveToPose(Swerve swerve, PoseAlignController controller, FieldPose pose, boolean hold) {
    m_swerve = swerve;
    targetPose = pose;
    holdPose = hold;
    m_controller = controller;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_swerve);
  }

  public DriveToPose(Swerve swerve, PoseAlignController controller, Pose2d pose, boolean hold) {
    m_swerve = swerve;
    targetPose2d = pose;
    holdPose = hold;
    m_controller = controller;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_controller.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    ChassisSpeeds speeds =
        m_controller.update(
            m_swerve.getPose(),
            targetPose == null
                ? targetPose2d
                : CheckBounds.getPose2DFromFieldPose(m_swerve, targetPose));
    m_swerve.setChassisSpeeds(speeds);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_swerve.setChassisSpeeds(new ChassisSpeeds(0, 0, 0));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return !holdPose && m_controller.atGoal();
  }
}
