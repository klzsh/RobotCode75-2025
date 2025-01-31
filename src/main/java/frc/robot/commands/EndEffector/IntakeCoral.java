// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.EndEffector;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.EndEffectorConstants;
import frc.robot.subsystems.EndEffector.CoralIntake;
import frc.robot.subsystems.EndEffector.CoralIntake.CoralStates;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class IntakeCoral extends Command {
  private final CoralIntake m_coralIntake;
  private double startTimestamp = -1;

  /** Creates a new IntakeCoral. */
  public IntakeCoral(CoralIntake coralIntake) {
    m_coralIntake = coralIntake;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_coralIntake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_coralIntake.setState(CoralStates.INTAKING);
    startTimestamp = -1;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (interrupted) {
      m_coralIntake.setState(CoralStates.DEFAULT);
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (m_coralIntake.getState() == CoralStates.HASGAMEPIECE && startTimestamp == -1) {
      startTimestamp = Timer.getFPGATimestamp();
      return false;
    }
    if (startTimestamp == -1) return false;
    return (Timer.getFPGATimestamp() - startTimestamp >= EndEffectorConstants.coralIntakeDelay);
}
}