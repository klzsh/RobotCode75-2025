// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.EndEffector;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.EndEffector.AlgaeIntake;
import frc.robot.subsystems.EndEffector.AlgaeIntake.AlgaeStates;
import frc.robot.subsystems.EndEffector.AlgaeIntake.PivotState;
import frc.robot.subsystems.EndEffector.Elevator;
import frc.robot.subsystems.EndEffector.Elevator.ElevatorPositions;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class DeAlgaefy extends SequentialCommandGroup {
  /** Creates a new DeAlgaefy. */
  // TODO: use a conditional command
  public DeAlgaefy(Elevator elevator, AlgaeIntake algaeIntake, boolean isL2) {
    if (isL2) { // temporary fix until we get april tags
      addCommands(
          /* L2 Algae Intake */
          elevator.positionCommand(ElevatorPositions.L2, true),
          new ParallelCommandGroup(
              Commands.runOnce(
                  () -> {
                    algaeIntake.setAlgaeState(AlgaeStates.INTAKING);
                    algaeIntake.setPivotState(PivotState.DEAGLAEFY);
                  }),
              elevator.positionCommand(ElevatorPositions.L2, false)),
          elevator.positionCommand(ElevatorPositions.HOME, false));
    } else {
      /* L3 Algae Intake */
      addCommands(
          elevator.positionCommand(ElevatorPositions.L3, true),
          new ParallelCommandGroup(
              Commands.runOnce(
                  () -> {
                    algaeIntake.setAlgaeState(AlgaeStates.INTAKING);
                    algaeIntake.setPivotState(PivotState.DEAGLAEFY);
                  }),
              elevator.positionCommand(ElevatorPositions.L3, false)),
          elevator.positionCommand(ElevatorPositions.HOME, false));
    }
  }
}
