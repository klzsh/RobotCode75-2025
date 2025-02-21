// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autonomous;

import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Drivetrain.DriveToPose;
import frc.robot.subsystems.Drivetrain.PoseAlignController;
import frc.robot.subsystems.Drivetrain.Swerve;
import frc.robot.subsystems.EndEffector.AlgaeIntake;
import frc.robot.subsystems.EndEffector.AlgaeIntake.AlgaeStates;
import frc.robot.subsystems.EndEffector.AlgaePivot;
import frc.robot.subsystems.EndEffector.AlgaePivot.PivotState;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AutoScoreProcessor extends SequentialCommandGroup {
  public AutoScoreProcessor(
      Swerve swerve, PoseAlignController poseController, AlgaeIntake intake, AlgaePivot pivot) {
    addRequirements(intake, pivot);
    int tagId = DriverStation.getAlliance().get() == Alliance.Blue ? 16 : 3;
    Pose2d tagPose =
        AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded)
            .getTagPose(tagId)
            .get()
            .toPose2d();
    Rotation2d tagHeading = tagPose.getRotation();
    Pose2d poseToDrive =
        tagPose.transformBy(
            new Transform2d(
                Inches.of(17.5 * tagHeading.getCos()),
                Inches.of(17.5 * tagHeading.getSin()),
                Rotation2d.fromDegrees(180)));
    addCommands(
        new DriveToPose(swerve, poseController, poseToDrive, false),
        new InstantCommand(
                () -> {
                  intake.setAlgaeState(AlgaeStates.OUTAKING);
                  pivot.setPivotState(PivotState.RETRACTED);
                })
            .repeatedly()
            .until(() -> !intake.algaeInIntake()),
        new InstantCommand(
            () -> {
              intake.setAlgaeState(AlgaeStates.NONE);
            }));
  }
}
