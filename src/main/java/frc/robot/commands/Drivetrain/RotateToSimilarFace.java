package frc.robot.commands.Drivetrain;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain.RotationController;
import frc.robot.subsystems.Drivetrain.Swerve;

public class RotateToSimilarFace extends Command {
  private final Swerve m_Swerve;
  private double targetHeading;
  private RotationController m_RotationController;

  public RotateToSimilarFace(Swerve swerve) {
    m_Swerve = swerve;
    m_RotationController = new RotationController(swerve);
    addRequirements(m_Swerve);
  }

  @Override
  public void initialize() {
    targetHeading = Math.round(m_Swerve.getRotation2D().getDegrees() / 60.0) * 60.0;
  }

  @Override
  public void execute() {
    m_RotationController.update(Rotation2d.fromDegrees(targetHeading));
    m_Swerve.setRobotRelative(new ChassisSpeeds(0, 0, m_RotationController.getOutput()));
  }

  @Override
  public void end(boolean interrupted) {
    m_Swerve.setRobotRelative(new ChassisSpeeds(0, 0, 0));
  }

  @Override
  public boolean isFinished() {
    return m_RotationController.atGoal();
  }
}
