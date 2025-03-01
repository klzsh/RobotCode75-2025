// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autonomous;

import static frc.robot.Constants.EndEffectorConstants.coralScoreDelay;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.lib.util.CheckBounds;
import frc.lib.util.FieldPose;
import frc.lib.util.FieldPose.Offset;
import frc.robot.commands.Drivetrain.AlignToBranch;
import frc.robot.commands.EndEffector.Coral.ScoreCoral;
import frc.robot.commands.EndEffector.SetElevatorPosition;
import frc.robot.subsystems.Drivetrain.PoseAlignController;
import frc.robot.subsystems.Drivetrain.Swerve;
import frc.robot.subsystems.EndEffector.CoralIntake;
import frc.robot.subsystems.EndEffector.Elevator;
import frc.robot.subsystems.EndEffector.Elevator.ElevatorPositions;
import frc.robot.subsystems.Vision.AprilTagCamera;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AutoScoreL4 extends SequentialCommandGroup {
  public AutoScoreL4(
      Swerve swerve,
      Elevator elevator,
      CoralIntake coralIntake,
      AprilTagCamera camera,
      PoseAlignController poseController,
      boolean isLeft) {
    addRequirements(swerve, elevator, coralIntake);
    addCommands(
        new ParallelCommandGroup(
            new AlignToBranch(
                swerve,
                camera,
                isLeft,
                poseController,
                new FieldPose(
                    DriverStation.getAlliance().get(),
                    CheckBounds.nearestElement(swerve.getPose()),
                    isLeft ? Offset.LEFT : Offset.RIGHT)),
            new SetElevatorPosition(elevator, ElevatorPositions.L4, true)),
        new ParallelCommandGroup(
            new ScoreCoral(coralIntake),
            new SetElevatorPosition(elevator, ElevatorPositions.L4, false)),
        new ParallelCommandGroup(
            new SetElevatorPosition(elevator, ElevatorPositions.L1, false),
            new WaitCommand(coralScoreDelay)),
        new SetElevatorPosition(elevator, ElevatorPositions.HOME, false));
  }
}
