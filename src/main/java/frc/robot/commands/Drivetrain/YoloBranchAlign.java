package frc.robot.commands.Drivetrain;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.subsystems.Drivetrain.Swerve;
import frc.robot.subsystems.Vision.ObjectDetetectorCamera;
import java.util.OptionalDouble;

public class YoloBranchAlign extends Command {

  // private final TunableNumber[] strafePID = {
  //   new TunableNumber("YOLO Align/P", 0.07),
  //   new TunableNumber("YOLO Align/I", 0),
  //   new TunableNumber("YOLO Align/D", 0),
  //   new TunableNumber("YOLO Align/Tolerance", 0.2)
  // };

  private final Swerve m_Swerve;
  private final ObjectDetetectorCamera m_BranchDetectorCamera;
  private final boolean isAlignInPlace;

  // private final PIDController rotationController; // add if needed, poss just pass through a
  // heading or do this as a seperate command
  // private final PIDController xController;
  private final PIDController yController;
  private double xCommand;
  private double yCommand;

  private OptionalDouble currentYaw;
  private OptionalDouble currentPitch;

  private ChassisSpeeds desiredSpeeds;

  // private final TunableNumber inPlaceYP;
  // private final TunableNumber inPlaceYD;

  private final double finalYawSetpointDegrees = -2.2;
  private final double driveIntoReefSpeed = .5;
  private final double stallSpeedThreshold = .05;
  double startTime = 0;

  public YoloBranchAlign(
      Swerve swerve, ObjectDetetectorCamera brachDetectorCamera, boolean alignInPlace) {
    m_Swerve = swerve;
    m_BranchDetectorCamera = brachDetectorCamera;
    isAlignInPlace = alignInPlace;

    yController =
        new PIDController(
            DrivetrainConstants.ControllerConstants.VisionAlign.xP,
            DrivetrainConstants.ControllerConstants.VisionAlign.xI,
            DrivetrainConstants.ControllerConstants.VisionAlign.xD);
    yController.setTolerance(.2);
    yController.setSetpoint(Math.sin(finalYawSetpointDegrees));
    if (alignInPlace) {
      yController.setP(0.1);
      yController.setD(0);
      yController.setTolerance(2);
    }

    desiredSpeeds = new ChassisSpeeds();

    addRequirements(m_Swerve);
  }

  @Override
  public void initialize() {
    startTime = Timer.getFPGATimestamp();
  }

  public double getAlignCommand() {
    double targetYaw = m_BranchDetectorCamera.getTargetYaw(0).getAsDouble();
    double targetSin = Math.sin(targetYaw);
    yCommand = yController.calculate(targetSin);
    return yCommand;
  }

  public void setChassisSpeeds(double xCommand, double yCommand) {
    desiredSpeeds.vxMetersPerSecond = xCommand;
    desiredSpeeds.vyMetersPerSecond = yCommand;
    m_Swerve.setChassisSpeeds(desiredSpeeds);
  }

  @Override
  public void execute() {
    // yController.setP(strafePID[0].getNumber());
    // yController.setI(strafePID[1].getNumber());
    // yController.setD(strafePID[2].getNumber());
    // yController.setTolerance(strafePID[3].getNumber());
    // yController.setP(0.07);
    // yController.setTolerance(0.2);

    SmartDashboard.putBoolean("YOLO yAtSetpoint", yController.atSetpoint());

    m_BranchDetectorCamera.updateByUnreadResults();

    if (!isAlignInPlace) {
      // has drive forward and strafe
      if (!m_BranchDetectorCamera.hasTargets()) {
        setChassisSpeeds(.1, 0);
      } else {
        // if we have a target, strafe to align with it
        yCommand = getAlignCommand();
        setChassisSpeeds(driveIntoReefSpeed, yCommand);
      }
    } else {
      // has strafe only
      if (!m_BranchDetectorCamera.hasTargets()) {
        // if we don't have a target, just go left
        setChassisSpeeds(0, -.05);
      } else {
        // if we have a target, strafe to align with it
        yCommand = getAlignCommand();
        setChassisSpeeds(0, yCommand);
      }
    }
  }

  @Override
  public boolean isFinished() {
    // should be stalling when driving into reef
    // return Timer.getFPGATimestamp() - startTime >= 3;
    // actual vx less than stall speed
    //
    if (!isAlignInPlace) {
      return m_Swerve.getChassisSpeeds().vxMetersPerSecond <= stallSpeedThreshold
          && Timer.getFPGATimestamp() - startTime >= 0.5;
    } else {
      return yController.atSetpoint();
    }
  }

  @Override
  public void end(boolean interrupted) {
    if (interrupted) {
      System.out.println("Ended, interrupted");
    } else {
      System.out.println("ended, not interrupted");
    }
    m_Swerve.stopModules();
  }
}
