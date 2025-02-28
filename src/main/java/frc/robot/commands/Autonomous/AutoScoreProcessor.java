// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autonomous;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.lib.util.CheckBounds;
import frc.lib.util.FieldPose;
import frc.lib.util.FieldPose.FieldElement;
import frc.lib.util.FieldPose.Offset;
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
    addRequirements(swerve, intake, pivot);
    Pose2d poseToDrive =
        CheckBounds.getPose2DFromFieldPose(
            swerve, new FieldPose(DriverStation.getAlliance().get(), FieldElement.P, Offset.MID));
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
