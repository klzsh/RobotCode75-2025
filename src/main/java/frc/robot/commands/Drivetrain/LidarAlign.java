package frc.robot.commands.Drivetrain;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain.Swerve;
import frc.robot.subsystems.Util.LidarDistance;

public class LidarAlign extends Command {
  private Swerve m_Swerve;
  private LidarDistance m_LidarDistance;
  private boolean left;
  private ChassisSpeeds speeds = new ChassisSpeeds();

  public LidarAlign(Swerve swerve, LidarDistance lidarDistance, boolean alignLeft) {
    m_Swerve = swerve;
    m_LidarDistance = lidarDistance;
    left = alignLeft;

    addRequirements(swerve);
  }

  @Override
  public void initialize() {
    if (left) {
      speeds = new ChassisSpeeds(0, .5, 0);
    } else {
      speeds = new ChassisSpeeds(0, -.5, 0);
    }
  }

  @Override
  public void execute() {
    m_Swerve.setChassisSpeeds(speeds);
  }

  @Override
  public void end(boolean interrupted) {
    m_Swerve.setChassisSpeeds(new ChassisSpeeds(0, 0, 0));
  }

  @Override
  public boolean isFinished() {
    return m_LidarDistance.belowThreshold();
  }
}
