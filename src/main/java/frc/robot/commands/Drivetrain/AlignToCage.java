package frc.robot.commands.Drivetrain;

import java.util.Optional;
import java.util.OptionalDouble;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain.RotationController;
import frc.robot.subsystems.Drivetrain.Swerve;
import frc.robot.subsystems.Vision.ObjectDetetectorCamera;

public class AlignToCage extends Command {

    private final Swerve m_Swerve;
    private final ObjectDetetectorCamera m_CageDetector;

    private final double finalYawSetpoint = -8;
    private final double finalPitchSetpoint = -9;
    private final double intermediateYawSetpoint = -3;  // only used if align needs to be two step

    private double yawSetpoint;

    private OptionalDouble currentYaw;
    private OptionalDouble currentPitch;

    private final PIDController xController;
    private final PIDController yController;
    private final PIDController rotationController;

    private double xCommand;
    private double yCommand;
    private double rotationCommand;

    public AlignToCage(Swerve swerve, ObjectDetetectorCamera cageDetector) {
        m_Swerve = swerve;
        m_CageDetector = cageDetector;

        xController = new PIDController(0.10, 0.0, 0.0);
        yController = new PIDController(0.10, 0.0, 0.0);
        rotationController = new PIDController(0.05, 0.0, 0.0);
        rotationController.setTolerance(1.5);

        addRequirements(m_Swerve);
    }

    @Override
    public void initialize() {
        m_CageDetector.updateByUnreadResults();
    }

    @Override
    public void execute() {
        m_CageDetector.updateByUnreadResults(); // updating here since updating periodically in subsystem is prob unnecessary 
        
        currentYaw = m_CageDetector.getTargetYaw(0);
        currentPitch = m_CageDetector.getTargetPitch(0);
        rotationCommand = rotationController.calculate(m_Swerve.getRotation2D().getDegrees(), 0);

        if (currentYaw.isPresent() && currentPitch.isPresent()) {
            yawSetpoint = -0.290486*(currentPitch.getAsDouble()) - 12.11571; // Linear regression
            xCommand = xController.calculate(currentPitch.getAsDouble(), finalPitchSetpoint);
            yCommand = yController.calculate(currentYaw.getAsDouble(), yawSetpoint);
            
            xCommand = MathUtil.clamp(xCommand, -1, 1); // not tryna fly away n shi
            yCommand = MathUtil.clamp(yCommand, -1, 1);

            m_Swerve.setChassisSpeeds(new ChassisSpeeds(xCommand, yCommand, rotationCommand));
        }
        else {
            m_Swerve.setChassisSpeeds(new ChassisSpeeds(0.3, 0, rotationCommand)); // creep forward
        }
    }

    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public boolean isFinished() {
        return false;
    }

    
}
