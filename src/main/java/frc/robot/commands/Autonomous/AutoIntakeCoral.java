// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autonomous;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.lib.util.CheckBounds;
import frc.lib.util.FieldPose;
import frc.lib.util.FieldPose.FieldElement;
import frc.lib.util.FieldPose.Offset;
import frc.robot.commands.Drivetrain.DriveToPose;
import frc.robot.commands.EndEffector.Coral.IntakeCoral;
import frc.robot.subsystems.Drivetrain.PoseAlignController;
import frc.robot.subsystems.Drivetrain.Swerve;
import frc.robot.subsystems.EndEffector.CoralIntake;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoIntakeCoral extends SequentialCommandGroup {
  /** Creates a new AutoIntakeCoral. */
  public AutoIntakeCoral(
      Swerve swerve,
      CoralIntake coralIntake,
      PoseAlignController poseAlignController,
      Offset offset) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addRequirements(swerve, coralIntake);
    FieldElement elem = CheckBounds.nearestElement(swerve.getPose());
    if (!FieldPose.fieldElementIsHPStation(elem)) {
      elem = FieldElement.HT;
    }
    addCommands(
        new DriveToPose(
            swerve,
            poseAlignController,
            new FieldPose(
                DriverStation.getAlliance().get(),
                elem,
                offset),
            false),
        new IntakeCoral(coralIntake));
  }
}
