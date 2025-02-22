// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drivetrain;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.lib.util.FieldPose;
import frc.robot.subsystems.Drivetrain.PoseAlignController;
import frc.robot.subsystems.Drivetrain.Swerve;
import frc.robot.subsystems.Drivetrain.VisionTranslationController;

public class DriveVisionAlign extends SequentialCommandGroup {

  public DriveVisionAlign(
      Swerve swerve,
      FieldPose targetPose,
      PoseAlignController poseController,
      VisionTranslationController visionController) {

    addCommands(
        Commands.runOnce(
            () -> {
              poseController.reset();
            }),
        new DriveToPose(swerve, poseController, targetPose, false),
        new VisionAlign(swerve, targetPose, visionController));
    addRequirements(swerve);
  }
}
