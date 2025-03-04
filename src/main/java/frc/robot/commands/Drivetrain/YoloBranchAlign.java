package frc.robot.commands.Drivetrain;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.dashboard.TunableNumber;
import frc.robot.subsystems.Drivetrain.Swerve;
import frc.robot.subsystems.Vision.ObjectDetetectorCamera;
import java.util.OptionalDouble;


public class YoloBranchAlign extends Command {

  private final TunableNumber[] strafePID = {
    new TunableNumber("YOLO Align/P", 0),
    new TunableNumber("YOLO Align/I", 0),
    new TunableNumber("YOLO Align/D", 0),
    new TunableNumber("YOLO Align/Tolderance", 0.05)
  };

  private final Swerve m_Swerve;
  private final ObjectDetetectorCamera m_BranchDetectorCamera;
  private final boolean isCenterAlign;

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
      Swerve swerve, ObjectDetetectorCamera brachDetectorCamera, boolean alignCenter) {
    m_Swerve = swerve;
    m_BranchDetectorCamera = brachDetectorCamera;
    isCenterAlign = alignCenter;

    xController = new PIDController(.1, 0, 0);
    yController = new PIDController(.1, 0, 0);
    yController.setTolerance(.01);
    yController.setSetpoint(Math.sin(Units.degreesToRadians(finalYawSetpoint)));

    desiredSpeeds = new ChassisSpeeds();

    addRequirements(m_Swerve);
  }


  @Override
  public void initialize() {}

  @Override
  public void execute() {
    yController.setP(strafePID[0].getNumber());
    yController.setI(strafePID[1].getNumber());
    yController.setD(strafePID[2].getNumber());
    yController.setTolerance(strafePID[3].getNumber());

    SmartDashboard.putBoolean("YOLO yatSetpoint", yController.atSetpoint());

    m_BranchDetectorCamera.updateByUnreadResults();

    if (!m_BranchDetectorCamera.hasTargets()) {
      m_Swerve.setChassisSpeeds(new ChassisSpeeds(0, 0, 0)); // poss creep
      return;
    }

    int targetTagID = 0;
    double targetYaw = 0;
    double secondTargetYaw = 0; // for center align
    double lowestError = Double.MAX_VALUE;
    double secondLowestError = Double.MAX_VALUE;
    for (int j = 0; j < m_BranchDetectorCamera.getNumTargets().getAsInt(); j++) {
      double currentYaw = Math.sin(Units.degreesToRadians(m_BranchDetectorCamera.getTargetYaw(j).getAsDouble()));
      double currentError = Math.abs(currentYaw - Math.sin(Units.degreesToRadians(finalYawSetpoint)));

      if (currentError < lowestError) {
        lowestError = currentError;
        targetTagID = j;
        targetYaw = currentYaw;

        if (isCenterAlign) {
          secondLowestError = lowestError;
          secondTargetYaw = targetYaw;
        }
      }

      if (isCenterAlign && currentError < secondLowestError && currentError != lowestError) {
        secondLowestError = currentError;
        secondTargetYaw = currentYaw;
      }
    }

    if (isCenterAlign) {
      // basically we are trying to equalize the yaw on either side of the center target
      targetYaw = (targetYaw + secondTargetYaw); // might need to be negated
    }

    yCommand = yController.calculate(targetYaw);

    if (!yController.atSetpoint()) {
      // if (isLeft) { // use leftmost for target indexing in photonvision
      //   currentYaw = m_BranchDetectorCamera.getTargetYaw(0);
      // } else {
      //   currentYaw =
      //       m_BranchDetectorCamera.getTargetYaw(
      //           m_BranchDetectorCamera.getNumTargets().getAsInt() - 1);
      // }
      desiredSpeeds = new ChassisSpeeds(0, yCommand, 0);
      m_Swerve.setChassisSpeeds(desiredSpeeds); // creep if needed
    } else {
      desiredSpeeds = new ChassisSpeeds(driveIntoReefSpeed, yCommand, 0);
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
