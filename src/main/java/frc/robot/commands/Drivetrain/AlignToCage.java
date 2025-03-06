package frc.robot.commands.Drivetrain;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain.Swerve;
import frc.robot.subsystems.Vision.ObjectDetetectorCamera;
import java.util.OptionalDouble;

public class AlignToCage extends Command {

  private final Swerve m_Swerve;
  private final ObjectDetetectorCamera m_CageDetector;

  private final double finalYawSetpoint = -8;
  private final double finalPitchSetpoint = -9;
  private final double intermediatePitchSetpoint = 0; // only used if align needs to be two step

  private double yawSetpoint;
  private double pitchSetpoint;

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

    xController = new PIDController(0.05, 0.0, 0.0);
    xController.setTolerance(.3);
    yController = new PIDController(0.05, 0.0, 0.0);
    yController.setTolerance(.3);
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
    m_CageDetector
        .updateByUnreadResults(); // updating here since updating periodically in subsystem is prob
    // unnecessary

    currentYaw = m_CageDetector.getTargetYaw(0);
    currentPitch = m_CageDetector.getTargetPitch(0);
    rotationCommand =
        rotationController.calculate(
            m_Swerve.getRotation2D().getDegrees(),
            180);

    if (currentYaw.isPresent() && currentPitch.isPresent()) {
      if (!yController.atSetpoint()) {
        xCommand = xController.calculate(currentPitch.getAsDouble(), intermediatePitchSetpoint);
      } else {
        xCommand = -1; // drive forward after aligned
      }

      yawSetpoint = -0.290486 * (currentPitch.getAsDouble()) - 12.11571; // Linear regression

      yCommand = yController.calculate(currentYaw.getAsDouble(), yawSetpoint);
      xCommand = MathUtil.clamp(xCommand, -1, 1); // not tryna fly away n shi
      yCommand = MathUtil.clamp(yCommand, -1, 1);

      m_Swerve.setChassisSpeeds(new ChassisSpeeds(xCommand, -yCommand, rotationCommand));
    } else {
      m_Swerve.setChassisSpeeds(new ChassisSpeeds(-0.3, 0, rotationCommand)); // creep forward
    }
  }

  @Override
  public void end(boolean interrupted) {
    m_Swerve.stopModules();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
