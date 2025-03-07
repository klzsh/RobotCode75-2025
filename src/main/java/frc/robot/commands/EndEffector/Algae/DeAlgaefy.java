// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.EndEffector.Algae;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.EndEffector.SetElevatorPosition;
import frc.robot.subsystems.EndEffector.AlgaeIntake;
import frc.robot.subsystems.EndEffector.AlgaeIntake.AlgaeStates;
import frc.robot.subsystems.EndEffector.AlgaePivot;
import frc.robot.subsystems.EndEffector.AlgaePivot.PivotState;
import frc.robot.subsystems.EndEffector.Elevator;
import frc.robot.subsystems.EndEffector.Elevator.ElevatorPositions;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class DeAlgaefy extends SequentialCommandGroup {
  /** Creates a new DeAlgaefy. */
  // TODO: use a conditional command
  public DeAlgaefy(
      Elevator elevator, AlgaeIntake algaeIntake, AlgaePivot algaePivot, boolean isL2) {
    if (isL2) { // temporary fix until we get april tags
      /*
       * first move elev to position
       *  parallel:
       *    {
       *    spin wheels
       *    extend pivot
       *      UNTIL: limit switch is pressed (button release)
       *    }
       *    {
       *      HOLD ELEV POSIITON
       *    }
       *    THEN:
       *     Sequence:
       *      switch state to has game peice
       *      retract pivot
       *  THEN
       *    lower elev
       */
      addRequirements(elevator);
      addCommands(
          new SetElevatorPosition(elevator, ElevatorPositions.L2, true),
          new ParallelCommandGroup(
                  new InstantCommand(
                          () -> {
                            algaeIntake.setAlgaeState(AlgaeStates.INTAKING);
                            algaePivot.setPivotState(PivotState.DEALGAEFY);
                          },
                          algaeIntake,
                          algaePivot)
                      .repeatedly()
                      .until(() -> algaeIntake.getAlgaeState() == AlgaeStates.HASGAMEPIECE),
                  new SetElevatorPosition(elevator, ElevatorPositions.L2, true))
              .until(() -> algaeIntake.getAlgaeState() == AlgaeStates.HASGAMEPIECE),
          new WaitCommand(0.2),
          new InstantCommand(() -> algaePivot.setPivotState(PivotState.RETRACTED))
              .repeatedly()
              .until(() -> algaePivot.isAtPosition(PivotState.RETRACTED)));
    } else {
      /* L3 Algae Intake */
      addCommands(
          new SetElevatorPosition(elevator, ElevatorPositions.L3, true),
          new ParallelCommandGroup(
                  new InstantCommand(
                          () -> {
                            algaeIntake.setAlgaeState(AlgaeStates.INTAKING);
                            algaePivot.setPivotState(PivotState.DEALGAEFY);
                          },
                          algaeIntake,
                          algaePivot)
                      .repeatedly()
                      .until(() -> algaeIntake.getAlgaeState() == AlgaeStates.HASGAMEPIECE),
                  new SetElevatorPosition(elevator, ElevatorPositions.L3, true))
              .until(() -> algaeIntake.getAlgaeState() == AlgaeStates.HASGAMEPIECE),
          new WaitCommand(0.2),
          new InstantCommand(() -> algaePivot.setPivotState(PivotState.RETRACTED))
              .repeatedly()
              .until(() -> algaePivot.isAtPosition(PivotState.RETRACTED)));
    }
  }
}
