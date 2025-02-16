// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.EndEffector;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.EndEffector.AlgaeIntake;
import frc.robot.subsystems.EndEffector.AlgaePivot;
import frc.robot.subsystems.EndEffector.AlgaeIntake.AlgaeStates;
import frc.robot.subsystems.EndEffector.AlgaePivot.PivotState;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class GroundAlgaePickup extends SequentialCommandGroup {
  /** Creates a new GroundAlgaePickup. */
  public GroundAlgaePickup(AlgaeIntake algaeIntake, AlgaePivot algaePivot) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands( new InstantCommand(
                          () -> {
                            algaeIntake.setAlgaeState(AlgaeStates.INTAKING);
                            algaePivot.setPivotState(PivotState.DEALGAEFY);
                          },
                          algaeIntake,
                          algaePivot)
                      .repeatedly()
                      .until(() -> algaeIntake.getAlgaeState() == AlgaeStates.HASGAMEPIECE));
  }
}
