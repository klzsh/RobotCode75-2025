// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drivetrain;

import static edu.wpi.first.units.Units.*;
import static frc.robot.Constants.FieldConstants.*;
import static frc.robot.Constants.VisionConstants.*;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.lib.util.FieldPose;
import frc.lib.util.FieldPose.FieldElement;
import frc.lib.util.FieldPose.Offset;
import frc.robot.subsystems.Drivetrain.PoseAlignController;
import frc.robot.subsystems.Drivetrain.Swerve;
import frc.robot.subsystems.Drivetrain.VisionTranslationController;

public class DriveVisionAlign extends SequentialCommandGroup {

  public DriveVisionAlign(
      Swerve swerve,
      int target,
      FieldPose targetPose,
      PoseAlignController poseController,
      VisionTranslationController visionController) {

    Pose2d tagPose =
        AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded)
            .getTagPose(target)
            .get()
            .toPose2d();
    Rotation2d tagHeading = tagPose.getRotation();
    Pose2d poseToDrive =
        tagPose.transformBy(
            new Transform2d(
                Inches.of(18 * -tagHeading.getCos()),
                Inches.of(18 * -tagHeading.getSin()),
                Rotation2d.fromDegrees(180)));
    if (FieldPose.fieldElementIsReef(targetPose.fieldElement) && targetPose.offset == Offset.LEFT) {
      tagHeading = tagHeading.rotateBy(Rotation2d.kCCW_90deg);
      poseToDrive = poseToDrive.transformBy(
              new Transform2d(
                  reefLeftPoseOffset.times(-tagHeading.getCos()),
                  reefLeftPoseOffset.times(-tagHeading.getSin()),
                Rotation2d.fromDegrees(0)
      ));
    }
    

    addCommands(
        Commands.runOnce(
            () -> {
              poseController.reset();
            }),
        new DriveToPose(swerve, poseController, poseToDrive, false),
        new VisionAlign(swerve, targetPose, visionController));
    addRequirements(swerve);
  }
}
