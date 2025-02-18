// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autonomous;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.EndEffector.AlgaeIntake;
import frc.robot.subsystems.EndEffector.AlgaeIntake.AlgaeStates;
import frc.robot.subsystems.EndEffector.AlgaePivot;
import frc.robot.subsystems.EndEffector.AlgaePivot.PivotState;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AutoScoreAlgae extends Command {
  /** Creates a new AutoScoreAlgae. */
  private final AlgaeIntake m_AlgaeIntake;

  private final AlgaePivot m_AlgaePivot;

  public AutoScoreAlgae(AlgaeIntake intake, AlgaePivot pivot) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_AlgaePivot = pivot;
    m_AlgaeIntake = intake;
    addRequirements(m_AlgaeIntake, m_AlgaePivot);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_AlgaePivot.setPivotState(PivotState.RETRACTED);
    m_AlgaeIntake.setAlgaeState(AlgaeStates.OUTAKING);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_AlgaeIntake.setAlgaeState(AlgaeStates.NONE);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return !m_AlgaeIntake.algaeInIntake();
  }
}
