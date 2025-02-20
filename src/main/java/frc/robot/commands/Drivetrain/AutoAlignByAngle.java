package frc.robot.commands.Drivetrain;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Drivetrain.Swerve;
import frc.robot.subsystems.Drivetrain.VisionTranslationController2;
import frc.robot.subsystems.Vision.AprilTagCamera;

public class AutoAlignByAngle extends SequentialCommandGroup {
  public AutoAlignByAngle(Swerve swerve, AprilTagCamera camera, VisionTranslationController2 visionController, boolean alignLeft) {
    addCommands(new SnapToNearestHeading(swerve), new TranslateToBranch(swerve, alignLeft, visionController));
    addRequirements(swerve);
  }
}
