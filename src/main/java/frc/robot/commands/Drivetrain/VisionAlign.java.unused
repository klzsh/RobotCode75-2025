package frc.robot.commands.Drivetrain;

import static frc.robot.Constants.VisionConstants.*;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.util.FieldPose;
import frc.lib.util.FieldPose.FieldElement;
import frc.robot.subsystems.Drivetrain.PoseAlignController;
import frc.robot.subsystems.Drivetrain.Swerve;
import frc.robot.subsystems.Drivetrain.VisionTranslationController;

public class VisionAlign extends Command {

  private final Swerve m_Swerve;
  private Translation2d offset;
  private boolean isLeft;
  private VisionTranslationController visionController;
  private PoseAlignController fallBackController;

  public VisionAlign(Swerve swerve, FieldPose targetPose, VisionTranslationController controller) {
    m_Swerve = swerve;

    if (FieldPose.fieldElementIsReef(targetPose.fieldElement)) {
      targetPose.fieldElement = FieldElement.A;
    }
    if (FieldPose.fieldElementIsHPStation(targetPose.fieldElement)) {
      targetPose.fieldElement = FieldElement.HT;
    }

    this.offset = fieldPoseToCameraAngleOffset.get(targetPose.toString());
    isLeft = targetPose.offset == FieldPose.Offset.LEFT;
    visionController = controller;

    addRequirements(m_Swerve);
  }

  @Override
  public void execute() {
    ChassisSpeeds speeds = visionController.update(this.offset, isLeft);
    m_Swerve.setChassisSpeeds(speeds);
  }

  @Override
  public void end(boolean interrupted) {
    m_Swerve.setChassisSpeeds(new ChassisSpeeds(0, 0, 0));
  }

  @Override
  public boolean isFinished() {
    return visionController.atGoal();
  }
}
