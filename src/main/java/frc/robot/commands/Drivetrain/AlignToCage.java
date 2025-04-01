package frc.robot.commands.Drivetrain;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.subsystems.Drivetrain.Swerve;
import frc.robot.subsystems.Vision.ObjectDetetectorCamera;
import java.util.OptionalDouble;

public class AlignToCage extends Command {

  private final Swerve m_Swerve;
  private final ObjectDetetectorCamera m_CageDetector;

  private final double intermediatePitchSetpoint = 0; // only used if align needs to be two step
  private final double finalPitchSetpoint = -5;

  private double yawSetpoint;

  private OptionalDouble currentYaw;
  private OptionalDouble currentPitch;

  private final PIDController xController;
  private final PIDController yController;
  private final PIDController rotationController;

  private double xCommand;
  private double yCommand;
  private double rotationCommand;
  private double rotationTarget;

  private double pitchTolerance;

  public AlignToCage(Swerve swerve, ObjectDetetectorCamera cageDetector) {
    m_Swerve = swerve;
    m_CageDetector = cageDetector;

    xController = new PIDController(0.05, 0.0, 0.0);
    xController.setTolerance(.7);
    yController = new PIDController(0.05, 0.0, 0.0);
    yController.setTolerance(pitchTolerance);
    rotationController = new PIDController(0.05, 0.0, 0.0);
    rotationController.setTolerance(1.5);

    addRequirements(m_Swerve);
  }

  @Override
  public void initialize() {
    m_CageDetector.updateByUnreadResults();
    if (DriverStation.getAlliance().get() == Alliance.Blue) { // orElse?
      rotationTarget = 180;
    } else {
      rotationTarget = 0;
    }
  }

  @Override
  public void execute() {
    m_CageDetector.updateByUnreadResults();

    currentYaw = m_CageDetector.getTargetYaw(0);
    currentPitch = m_CageDetector.getTargetPitch(0);

    if (Math.abs(m_Swerve.getRotation2D().getDegrees() - rotationTarget) <= 1.5) {
      rotationCommand =
          rotationController.calculate(m_Swerve.getRotation2D().getDegrees(), rotationTarget);
    } else {
      rotationCommand = 0;
    }

    if (currentYaw.isPresent() && currentPitch.isPresent()) {
      yawSetpoint = -0.290486 * (currentPitch.getAsDouble()) - 12.11571; // Linear regression
      if (Math.abs(currentYaw.getAsDouble() - yawSetpoint)
          > 5) { // allowed error prob should be quite large so it doesn't go in and out of this
        // block
        xCommand = xController.calculate(currentPitch.getAsDouble(), intermediatePitchSetpoint);
      } else {
        xCommand =
            xController.calculate(
                currentPitch.getAsDouble(), finalPitchSetpoint); // drive forward after aligned
      }

      yCommand = yController.calculate(currentYaw.getAsDouble(), yawSetpoint);
      xCommand = MathUtil.clamp(xCommand, -1, 1); // not tryna fly away n shi
      yCommand = MathUtil.clamp(yCommand, -1, 1);

      m_Swerve.setChassisSpeeds(new ChassisSpeeds(xCommand, -yCommand, rotationCommand));
    } else {
      xCommand *= .6;
      yCommand *= .6;
      m_Swerve.setChassisSpeeds(
          new ChassisSpeeds(xCommand, -yCommand, rotationCommand)); // creep forward
    }
  }

  @Override
  public void end(boolean interrupted) {
    m_Swerve.setChassisSpeeds(new ChassisSpeeds(0, 0, 0));
    SwerveModuleState[] states =
        DrivetrainConstants.swerveKinematics.toSwerveModuleStates(new ChassisSpeeds(0, 0, 0));

    // set all modules forward so that wheels can rotate and robot can move during winch
    states[0].angle = Rotation2d.fromDegrees(0);
    states[1].angle = Rotation2d.fromDegrees(0);
    states[3].angle = Rotation2d.fromDegrees(0);
    states[2].angle = Rotation2d.fromDegrees(0);
    for (SwerveModuleState state : states) {
      state.speedMetersPerSecond = 0;
    }
    m_Swerve.setModuleStates(states, true);

    // could add coast but might be OD and unnecessary
  }

  @Override
  public boolean isFinished() {
    return (currentPitch.isPresent()
        && currentYaw.isPresent()
        && Math.abs(currentPitch.getAsDouble() - finalPitchSetpoint) < pitchTolerance);
  }
}
