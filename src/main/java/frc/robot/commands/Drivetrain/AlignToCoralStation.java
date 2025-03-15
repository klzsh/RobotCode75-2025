package frc.robot.commands.Drivetrain;

import static frc.robot.Constants.DrivetrainConstants.swerveKinematics;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.util.FieldPose;
import frc.lib.util.FieldPose.Offset;
import frc.lib.util.PeddieBounds;
import frc.robot.subsystems.Drivetrain.ChezyController;
import frc.robot.subsystems.Drivetrain.Swerve;

public class AlignToCoralStation extends Command {
  private final Swerve m_Swerve;
  private final ChezyController m_ChezyController;
  private Pose2d targetPose;

  SwerveModuleState[] states = swerveKinematics.toSwerveModuleStates(new ChassisSpeeds(0, 0, 0));

  public AlignToCoralStation(Swerve swerve, ChezyController chezyController) {
    m_Swerve = swerve;
    m_ChezyController = chezyController;
    addRequirements(m_Swerve);
  }

  @Override
  public void initialize() {
    Alliance alliance = DriverStation.getAlliance().get(); // Default to Blue if not set
    targetPose = m_Swerve.getPose();

    targetPose =
        PeddieBounds.fieldElementToPose2d(
            m_Swerve, new FieldPose(alliance, PeddieBounds.getHPElement(m_Swerve), Offset.MID));

    m_ChezyController.reset(targetPose);
  }

  @Override
  public void execute() {
    ChassisSpeeds speeds = m_ChezyController.update(targetPose);
    m_Swerve.setFieldRelative(speeds);
  }

  @Override
  public void end(boolean interrupted) {
    m_Swerve.setChassisSpeeds(new ChassisSpeeds(0, 0, 0));

    // xStance but not from command cuz that shi don't end
    states[0].angle = Rotation2d.fromDegrees(45);
    states[1].angle = Rotation2d.fromDegrees(315);
    states[3].angle = Rotation2d.fromDegrees(225);
    states[2].angle = Rotation2d.fromDegrees(135);
    for (SwerveModuleState state : states) {
      state.speedMetersPerSecond = 0;
    }

    m_Swerve.setModuleStates(states, true);
  }

  @Override
  public boolean isFinished() {
    return m_ChezyController.isFinished();
  }
}
