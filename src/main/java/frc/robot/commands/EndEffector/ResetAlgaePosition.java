package frc.robot.commands.EndEffector;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.EndEffector.AlgaeIntake;

public class ResetAlgaePosition extends Command {
    private final AlgaeIntake m_algaeIntake;

    public ResetAlgaePosition(AlgaeIntake intake) {
        m_algaeIntake = intake;
        addRequirements(m_algaeIntake);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        m_algaeIntake.homePivotToAbsoluteEncoder();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        m_Elevator.setPosition(m_Position, m_IsAlgae);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return m_Elevator.isAtPosition(m_Position, m_IsAlgae);
    }
}
