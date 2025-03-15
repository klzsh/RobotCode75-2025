package frc.robot.commands.Drivetrain;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain.Swerve;
import frc.robot.subsystems.Vision.YoloController;

public class YoloBranchAlign extends Command {
  private final Swerve m_Swerve;
  private final YoloController m_YoloController;
  private boolean isAlignInPlace = false;

  public YoloBranchAlign(Swerve swerve, YoloController yoloController, boolean alignInPlace) {
    m_Swerve = swerve;
    m_YoloController = yoloController;
    isAlignInPlace = alignInPlace;

    addRequirements(m_Swerve);
  }

  @Override
  public void initialize() {
    m_YoloController.reset(isAlignInPlace);
  }

  @Override
  public void execute() {
    ChassisSpeeds speeds = m_YoloController.update();
    m_Swerve.setRobotRelative(speeds);
  }

  @Override
  public boolean isFinished() {
    return m_YoloController.atGoal();
  }

  @Override
  public void end(boolean interrupted) {
    if (interrupted) {
      System.out.println("Ended, interrupted");
    } else {
      System.out.println("ended, not interrupted");
    }
    m_Swerve.stopModules();
  }
}
