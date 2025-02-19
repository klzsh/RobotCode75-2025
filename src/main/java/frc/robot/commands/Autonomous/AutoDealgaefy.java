// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autonomous;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.lib.util.FieldPose;
import frc.lib.util.FieldPose.FieldElement;
import frc.lib.util.FieldPose.Offset;
import frc.robot.commands.Drivetrain.VisionAlign;
import frc.robot.commands.EndEffector.SetElevatorPosition;
import frc.robot.subsystems.Drivetrain.RotationController;
import frc.robot.subsystems.Drivetrain.Swerve;
import frc.robot.subsystems.Drivetrain.VisionTranslationController;
import frc.robot.subsystems.EndEffector.AlgaeIntake;
import frc.robot.subsystems.EndEffector.AlgaeIntake.AlgaeStates;
import frc.robot.subsystems.EndEffector.AlgaePivot;
import frc.robot.subsystems.EndEffector.AlgaePivot.PivotState;
import frc.robot.subsystems.EndEffector.Elevator;
import frc.robot.subsystems.EndEffector.Elevator.ElevatorPositions;
import frc.robot.subsystems.Vision.AprilTagCamera;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoDealgaefy extends SequentialCommandGroup {
  /** Creates a new AutoDealgaefy. */
  public AutoDealgaefy(
      Swerve swerve,
      Elevator elevator,
      AlgaeIntake intake,
      AlgaePivot pivot,
      AprilTagCamera centerCamera,
      String reefPoint,
      VisionTranslationController visionController,
      RotationController rotationController) {
    addRequirements(swerve, elevator, intake, pivot);
    int tagID =
        DriverStation.getAlliance().get() == Alliance.Blue
            ? 1
            : 2; // TODO make string to tag id maps
    ElevatorPositions elevatorHeight =
        reefPoint == "rl"
            ? ElevatorPositions.L2
            : ElevatorPositions.L3; // TODO make string to algae height maps
    addCommands(
        new ParallelCommandGroup(
            new VisionAlign(
                swerve,
                centerCamera,
                tagID,
                new FieldPose(DriverStation.getAlliance().get(), FieldElement.RL, Offset.MID),
                visionController,
                rotationController),
            new SetElevatorPosition(elevator, elevatorHeight, true)),
        new ParallelCommandGroup(
                new InstantCommand(
                        () -> {
                          intake.setAlgaeState(AlgaeStates.INTAKING);
                          pivot.setPivotState(PivotState.DEALGAEFY);
                        },
                        intake,
                        pivot)
                    .repeatedly()
                    .until(() -> intake.getAlgaeState() == AlgaeStates.HASGAMEPIECE),
                new SetElevatorPosition(elevator, elevatorHeight, true))
            .until(() -> intake.getAlgaeState() == AlgaeStates.HASGAMEPIECE),
        new InstantCommand(() -> pivot.setPivotState(PivotState.RETRACTED))
            .repeatedly()
            .until(() -> pivot.isAtPosition(PivotState.RETRACTED)));
  }
}
