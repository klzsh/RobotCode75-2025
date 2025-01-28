// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.EndEffector.Coral;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.EndEffector.SetElevatorPosition;
import frc.robot.subsystems.EndEffector.CoralIntake;
import frc.robot.subsystems.EndEffector.CoralIntake.CoralStates;
import frc.robot.subsystems.EndEffector.Elevator;
import frc.robot.subsystems.EndEffector.Elevator.ElevatorPositions;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ScoreL3 extends SequentialCommandGroup {
  /** Creates a new ScoreL3. */
  public ScoreL3(Elevator elevator, CoralIntake coralIntake) {
    addRequirements(elevator, coralIntake);
    addCommands(
        new SetElevatorPosition(elevator, ElevatorPositions.L1, false),
        new ParallelCommandGroup(
            new ParallelCommandGroup(
                new InstantCommand(() -> coralIntake.setState(CoralStates.SCORING), coralIntake),
                new WaitCommand(1)),
            new SetElevatorPosition(elevator, ElevatorPositions.L1, false)),
        new SetElevatorPosition(elevator, ElevatorPositions.HOME, false));
  }
}
