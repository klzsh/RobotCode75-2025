// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drivetrain;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.util.PeddieBounds;
import frc.lib.util.FieldPose;
import frc.robot.subsystems.Drivetrain.ChezyController;
import frc.robot.subsystems.Drivetrain.Swerve;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ChezyPose extends Command {
  private Swerve m_swerve;
  private FieldPose targetPose = null;
  private Pose2d targetPose2d = null;
  private boolean holdPose;
  private final ChezyController m_chezyController;

  /** Creates a new ChezyPose. */
  public ChezyPose(Swerve swerve, ChezyController controller, FieldPose pose, boolean hold) {
    m_swerve = swerve;
    m_chezyController = controller;
    targetPose = pose;
    holdPose = hold;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_swerve);
  }

  public ChezyPose(Swerve swerve, ChezyController controller, Pose2d pose, boolean hold) {
    m_swerve = swerve;
    m_chezyController = controller;
    targetPose2d = pose;
    holdPose = hold;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (targetPose2d == null) {
      targetPose2d = PeddieBounds.getNearestFieldPose2d(m_swerve, targetPose);
    }
    m_chezyController.reset(targetPose2d);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    ChassisSpeeds speeds = m_chezyController.update(targetPose2d);
    m_swerve.setFieldRelative(speeds);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_swerve.setChassisSpeeds(new ChassisSpeeds(0, 0, 0));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_chezyController.isFinished() && !holdPose;
  }
}
