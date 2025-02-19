// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drivetrain;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Drivetrain.RotationController;
import frc.robot.subsystems.Drivetrain.Swerve;
import frc.robot.subsystems.Drivetrain.VisionTranslationController;
import frc.robot.subsystems.Vision.AprilTagCamera;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class VisionAlign extends SequentialCommandGroup {

  public VisionAlign(
      Swerve swerve,
      AprilTagCamera camera,
      int target,
      VisionTranslationController visionController,
      RotationController rotationController) {
    /**
     * first snap rotation to april tag based on bounding box then calculate translation using
     * visioncontroller and rotation to preset heading using rotationcontroller modify chassisspeeds
     * from visioncontroller using rotationcontroller output set chassisspeeds
     */
    Rotation2d placeholder = Rotation2d.fromDegrees(0); // from bounding box
    Pose2d tagPose =
        AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded)
            .getTagPose(target)
            .get()
            .toPose2d();
    Pose2d currentPose = swerve.getPose();
    Rotation2d toTag =
        Rotation2d.fromRadians(
            Math.atan2(tagPose.getY() - currentPose.getY(), tagPose.getX() - currentPose.getX()));
    addCommands(
        Commands.runOnce(
            () -> {
              visionController.reset();
            }),
        new SnapHoldRotation(swerve, toTag, () -> 0, () -> 0),
        new InstantCommand(
                () -> {
                  ChassisSpeeds speeds = visionController.update(camera, target);
                  rotationController.update(placeholder);
                  speeds.omegaRadiansPerSecond = rotationController.getOutput();
                  swerve.setChassisSpeeds(speeds);
                })
            .repeatedly()
            .until(() -> visionController.atGoal() && rotationController.atGoal()));
    addRequirements(swerve);
  }
}
