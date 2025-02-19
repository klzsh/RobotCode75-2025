// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autonomous;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.lib.util.FieldPose;
import frc.lib.util.FieldPose.FieldElement;
import frc.lib.util.FieldPose.Offset;
import frc.robot.commands.Drivetrain.VisionAlign;
import frc.robot.commands.EndEffector.Coral.ScoreCoral;
import frc.robot.commands.EndEffector.SetElevatorPosition;
import frc.robot.subsystems.Drivetrain.RotationController;
import frc.robot.subsystems.Drivetrain.Swerve;
import frc.robot.subsystems.Drivetrain.VisionTranslationController;
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
      AprilTagCamera leftCamera,
      AprilTagCamera centerCamera,
      VisionTranslationController visionController,
      RotationController rotationController,
      boolean isLeft) {
    addRequirements(swerve, elevator, coralIntake);
    int tagID =
        DriverStation.getAlliance().get() == Alliance.Blue
            ? 1
            : 2; // TODO make string to tag id maps
    addCommands(
        new ParallelCommandGroup(
            new VisionAlign(
                swerve,
                isLeft ? centerCamera : leftCamera,
                tagID,
                new FieldPose(
                    DriverStation.getAlliance().get(),
                    FieldElement.RL,
                    isLeft ? Offset.LEFT : Offset.RIGHT),
                visionController,
                rotationController),
            new SetElevatorPosition(elevator, ElevatorPositions.L4, true)),
        new ParallelCommandGroup(
            new ScoreCoral(coralIntake),
            new SetElevatorPosition(elevator, ElevatorPositions.L4, false)),
        new SetElevatorPosition(elevator, ElevatorPositions.HOME, false));
  }
}
