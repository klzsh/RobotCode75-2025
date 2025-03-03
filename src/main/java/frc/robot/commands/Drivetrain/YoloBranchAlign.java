package frc.robot.commands.Drivetrain;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain.Swerve;
import frc.robot.subsystems.Vision.ObjectDetetectorCamera;
import java.util.OptionalDouble;

public class YoloBranchAlign extends Command {
  private final Swerve m_Swerve;
  private final ObjectDetetectorCamera m_BranchDetectorCamera;
  private final boolean isLeft;

  // private final PIDController rotationController; // add if needed, poss just pass through a
  // heading or do this as a seperate command
  private final PIDController xController;
  private final PIDController yController;
  private double xCommand;
  private double yCommand;

  private OptionalDouble currentYaw;
  private OptionalDouble currentPitch;

  private ChassisSpeeds desiredSpeeds;

  private final double finalYawSetpoint = 0;
  private final double driveIntoReefSpeed = .5;
  private final double stallSpeedThreshold = .05;

  public YoloBranchAlign(
      Swerve swerve, ObjectDetetectorCamera brachDetectorCamera, boolean alignLeft) {
    m_Swerve = swerve;
    m_BranchDetectorCamera = brachDetectorCamera;
    isLeft = alignLeft;

    xController = new PIDController(.1, 0, 0);
    yController = new PIDController(.1, 0, 0);
    yController.setTolerance(.1);
    yController.setSetpoint(Math.sin(Units.degreesToRadians(finalYawSetpoint)));

    desiredSpeeds = new ChassisSpeeds();

    addRequirements(m_Swerve);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    m_BranchDetectorCamera.updateByUnreadResults();

    if (!m_BranchDetectorCamera.hasTargets()) {
      m_Swerve.setChassisSpeeds(new ChassisSpeeds(0, 0, 0)); // poss creep
      return;
    }
    if (!yController.atSetpoint()) {
      if (isLeft) { // use leftmost for target indexing in photonvision
        currentYaw = m_BranchDetectorCamera.getTargetYaw(0);
      } else {
        currentYaw =
            m_BranchDetectorCamera.getTargetYaw(
                m_BranchDetectorCamera.getNumTargets().getAsInt() - 1);
      }

      yCommand = yController.calculate(Math.sin(Units.degreesToRadians(currentYaw.getAsDouble())));

      desiredSpeeds = new ChassisSpeeds(0, yCommand, 0);
      m_Swerve.setChassisSpeeds(desiredSpeeds); // creep if needed
    } else {
      desiredSpeeds = new ChassisSpeeds(driveIntoReefSpeed, 0, 0);
      m_Swerve.setChassisSpeeds(desiredSpeeds); // drive into reef once strafe aligned
    }
  }

  @Override
  public boolean isFinished() {
    // should be stalling when driving into reef
    return (m_Swerve.getChassisSpeeds().vxMetersPerSecond < stallSpeedThreshold
        && desiredSpeeds.vxMetersPerSecond == driveIntoReefSpeed);
  }

  @Override
  public void end(boolean interrupted) {
    m_Swerve.stopModules();
  }
}
