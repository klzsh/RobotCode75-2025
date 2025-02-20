package frc.robot.commands.Drivetrain;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain.Swerve;

public class SnapToNearestHeading extends Command {

    private final Swerve m_Swerve;
    private double targetHeading;
    private double currentHeading;
    private final double interval;
    
    public SnapToNearestHeading(Swerve swerve) {
        m_Swerve = swerve;
        interval = Math.PI / 3;
        addRequirements(m_Swerve);
    }
    
    @Override
    public void initialize() {   
        currentHeading = m_Swerve.getPose().getRotation().getRadians();
        targetHeading = Math.round(currentHeading / interval) * interval;
    }
    
    @Override
    public void execute() {
        m_Swerve.drive(new Translation2d(0, 0), targetHeading);
    }
    
    @Override
    public void end(boolean interrupted) {

    }
    
    @Override
    public boolean isFinished() {
        return Math.abs(m_Swerve.getPose().getRotation().getRadians() - targetHeading) <= 0.09;
    }
}
