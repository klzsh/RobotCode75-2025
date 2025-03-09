package frc.robot.commands.Drivetrain;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain.Swerve;

public class RotateToSimilarFace extends Command {
  private final Swerve m_Swerve;
  private double targetHeading;
  private PIDController m_RotationController;

  public RotateToSimilarFace(Swerve swerve) {
    m_Swerve = swerve;
    m_RotationController = new PIDController(0.05, 0, 0);
    m_RotationController.setTolerance(1.5);    
    addRequirements(m_Swerve);
  }


  @Override
  public void initialize() {
    targetHeading = Math.round(m_Swerve.getRotation2D().getDegrees() / 60.0) * 60.0;
  }

  @Override
  public void execute() {
    double rotationOutput =
        m_RotationController.calculate(m_Swerve.getRotation2D().getDegrees(), targetHeading);
    m_Swerve.setRobotRelative(new ChassisSpeeds(0, 0, rotationOutput));
  }

  @Override
  public void end(boolean interrupted) {
    m_Swerve.setRobotRelative(new ChassisSpeeds(0, 0, 0));
  }

  @Override
  public boolean isFinished() {
    return m_RotationController.atSetpoint();
  }
}
