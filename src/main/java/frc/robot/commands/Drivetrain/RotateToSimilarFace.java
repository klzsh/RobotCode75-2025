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
  private boolean hp;

  public RotateToSimilarFace(Swerve swerve) {
    m_Swerve = swerve;
    m_RotationController = new PIDController(0.05, 0, 0);
    m_RotationController.setTolerance(1.5);
    hp = false;
    addRequirements(m_Swerve);
  }

  public RotateToSimilarFace(Swerve swerve, boolean isHP) {
    m_Swerve = swerve;
    m_RotationController = new PIDController(0.05, 0, 0);
    m_RotationController.setTolerance(1.5);
    m_RotationController.enableContinuousInput(-180, 180);
    hp = isHP;
    addRequirements(m_Swerve);
  }

  private double wrap(double angle) {
    if (angle < -180) {
      return angle + 360;
    }
    if (angle > 180) {
      return angle - 360;
    }
    return angle;
  }

  @Override
  public void initialize() {
    targetHeading = Math.round(m_Swerve.getRotation2D().getDegrees() / 60.0) * 60.0;

    if (hp && DriverStation.getAlliance().get() == Alliance.Blue) {
      if (Math.abs(wrap(m_Swerve.getRotation2D().getDegrees() - 36))
          < Math.abs(wrap(m_Swerve.getRotation2D().getDegrees() - 144))) {
        targetHeading = 36;
      } else {
        targetHeading = 144;
      }
    } else if (hp) {
      if (Math.abs(wrap(m_Swerve.getRotation2D().getDegrees() - (36 - 180)))
          < Math.abs(wrap(m_Swerve.getRotation2D().getDegrees() - (144 - 180)))) {
        targetHeading = 36 - 180;
      } else {
        targetHeading = 144 - 180;
      }
    }
    if (targetHeading < -180) {
      targetHeading += 360;
    }
    if (targetHeading > 180) {
      targetHeading -= 360;
    }
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
