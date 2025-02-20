package frc.robot.commands.Drivetrain;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Drivetrain.Swerve;
import frc.robot.subsystems.Vision.AprilTagCamera;

public class AutoAlignByAngle extends SequentialCommandGroup {
  public AutoAlignByAngle(Swerve swerve, AprilTagCamera camera, boolean alignLeft) {
    addCommands(new TranslateToBranch(swerve, camera, alignLeft), new RotateToReef(swerve, camera));
    addRequirements(swerve);
  }
}
