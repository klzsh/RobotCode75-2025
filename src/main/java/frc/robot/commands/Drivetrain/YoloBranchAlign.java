package frc.robot.commands.Drivetrain;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.dashboard.TunableNumber;
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
  private final PIDController xController;
  private final PIDController yController;
  private double xCommand;
  private double yCommand;

  private OptionalDouble currentYaw;
  private OptionalDouble currentPitch;

  private ChassisSpeeds desiredSpeeds;

  private final double finalYawSetpoint = -2.2;
  private final double driveIntoReefSpeed = .5;
  private final double stallSpeedThreshold = .05;
  double startTime = 0;

  public YoloBranchAlign(
      Swerve swerve, ObjectDetetectorCamera brachDetectorCamera, boolean alignInPlace) {
    m_Swerve = swerve;
    m_BranchDetectorCamera = brachDetectorCamera;
    isAlignInPlace = alignInPlace;

    xController = new PIDController(.1, 0, 0);
    yController = new PIDController(0.07, 0, 0);
    yController.setTolerance(.2);
    yController.setSetpoint(finalYawSetpoint);

    desiredSpeeds = new ChassisSpeeds();

    addRequirements(m_Swerve);
  }

  @Override
  public void initialize() {
    startTime = Timer.getFPGATimestamp();
  }

  @Override
  public void execute() {
    // yController.setP(strafePID[0].getNumber());
    // yController.setI(strafePID[1].getNumber());
    // yController.setD(strafePID[2].getNumber());
    // yController.setTolerance(strafePID[3].getNumber());
    yController.setP(0.07);
    yController.setTolerance(0.2);

    SmartDashboard.putBoolean("YOLO yAtSetpoint", yController.atSetpoint());

    m_BranchDetectorCamera.updateByUnreadResults();

    if (!isAlignInPlace) {
      if (!m_BranchDetectorCamera.hasTargets()) {
        desiredSpeeds = new ChassisSpeeds(.1,0,0);
        m_Swerve.setChassisSpeeds(desiredSpeeds); // poss creep
      } else {
      double targetYaw = m_BranchDetectorCamera.getTargetYaw(0).getAsDouble();

      yCommand = yController.calculate(targetYaw);

      m_Swerve.setChassisSpeeds(new ChassisSpeeds(driveIntoReefSpeed, yCommand, 0));
      desiredSpeeds.vxMetersPerSecond = driveIntoReefSpeed;
      desiredSpeeds.vyMetersPerSecond = yCommand;
      }
    } 

    else {
      if (!m_BranchDetectorCamera.hasTargets()) {
        desiredSpeeds = new ChassisSpeeds(0,0.05,0); // scoot left n shi poss see a branch or sum
        m_Swerve.setChassisSpeeds(desiredSpeeds);
      }
      else {
        double targetYaw = m_BranchDetectorCamera.getTargetYaw(0).getAsDouble();

        yCommand = yController.calculate(targetYaw);
        desiredSpeeds.vxMetersPerSecond = 0; // can't push into reef while trying to align
        desiredSpeeds.vyMetersPerSecond = yCommand;
        m_Swerve.setChassisSpeeds(desiredSpeeds);
      }
    }
    // if (!yController.atSetpoint()) {
    //   // if (isLeft) { // use leftmost for target indexing in photonvision
    //   //   currentYaw = m_BranchDetectorCamera.getTargetYaw(0);
    //   // } else {
    //   //   currentYaw =
    //   //       m_BranchDetectorCamera.getTargetYaw(
    //   //           m_BranchDetectorCamera.getNumTargets().getAsInt() - 1);
    //   // }
    //   desiredSpeeds = new ChassisSpeeds(0, yCommand, 0);
    //   m_Swerve.setChassisSpeeds(desiredSpeeds); // creep if needed
    // } else {
    //   desiredSpeeds = new ChassisSpeeds(driveIntoReefSpeed, yCommand, 0);
    //   m_Swerve.setChassisSpeeds(desiredSpeeds); // drive into reef once strafe aligned
    // }
  }

  @Override
  public boolean isFinished() {
    // should be stalling when driving into reef
    // return Timer.getFPGATimestamp() - startTime >= 3;
    // actual vx less than stall speed
    // 
    if (!isAlignInPlace) {
    return m_Swerve.getChassisSpeeds().vxMetersPerSecond <= stallSpeedThreshold && Timer.getFPGATimestamp() - startTime >= 0.5;
    }
    else {
      return yController.atSetpoint();
    }
  }

  @Override
  public void end(boolean interrupted) {
    if(interrupted){
      System.out.println("Ended, interrupted");
    } else {
      System.out.println("ended, not interrupted");
    }
    m_Swerve.stopModules();
  }
}
