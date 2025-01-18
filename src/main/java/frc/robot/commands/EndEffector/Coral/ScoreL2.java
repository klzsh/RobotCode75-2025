// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.EndEffector.Coral;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.EndEffector.CoralIntake;
import frc.robot.subsystems.EndEffector.CoralIntake.CoralStates;
import frc.robot.subsystems.EndEffector.Elevator;
import frc.robot.subsystems.EndEffector.Elevator.ElevatorPositions;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ScoreL2 extends SequentialCommandGroup {
  /** Creates a new ScoreL2. */
  public ScoreL2(Elevator elevator, CoralIntake coralIntake) {
    addCommands(
        elevator.positionCommand(ElevatorPositions.L2, false),
        new ParallelCommandGroup(
            Commands.runOnce(() -> coralIntake.setState(CoralStates.SCORING)),
            elevator.positionCommand(ElevatorPositions.L2, false)),
        elevator.positionCommand(ElevatorPositions.HOME, false));
  }
}
