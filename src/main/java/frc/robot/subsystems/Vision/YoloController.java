// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Vision;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Importance;
import edu.wpi.first.epilogue.Logged.Strategy;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.dashboard.TunableNumber;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.subsystems.Drivetrain.Swerve;
@Logged(name = "YOLO Controller", importance = Importance.DEBUG, strategy = Strategy.OPT_IN)
public class YoloController {
  private final TunableNumber[] strafePID = {
    new TunableNumber("YOLO Align/P", 0.08),
    new TunableNumber("YOLO Align/I", 0),
    new TunableNumber("YOLO Align/D", 0.005),
    new TunableNumber("YOLO Align/Tolerance", 0.025)
  };

  private final Swerve m_Swerve;
  private final ObjectDetetectorCamera m_BranchDetectorCamera;
  private boolean isAlignInPlace;

  // private final PIDController rotationController; // add if needed, poss just pass through a
  // heading or do this as a seperate command
  // private final PIDController xController;
  private final PIDController yController;
  // private double xCommand;
  @Logged
  private double yCommand;

  // private final TunableNumber inPlaceYP;
  // private final TunableNumber inPlaceYD;

  private final double finalYawSetpointDegrees = -1.1;
  private final double driveIntoReefSpeed = .5;
  private final double stallSpeedThreshold = .05;
  double startTime = -1;

  /** Creates a new YoloController. */
  public YoloController(Swerve swerve, ObjectDetetectorCamera branchCam) {
    m_Swerve = swerve;
    m_BranchDetectorCamera = branchCam;
    yController =
        new PIDController(
            DrivetrainConstants.ControllerConstants.VisionAlign.xP,
            DrivetrainConstants.ControllerConstants.VisionAlign.xI,
            DrivetrainConstants.ControllerConstants.VisionAlign.xD);

    yController.setTolerance(.2);
    yController.setSetpoint(Math.sin(finalYawSetpointDegrees));
  }

  public double getAlignCommand() {
    double targetYaw = m_BranchDetectorCamera.getTargetYaw(0).getAsDouble();
    double targetSin = Math.sin(targetYaw);
    yCommand = yController.calculate(targetSin) * -Math.signum(targetYaw);
    return yCommand;
  }

  public void reset(boolean alignInPlace) {
    isAlignInPlace = alignInPlace;
    if (isAlignInPlace) {
      yController.setP(0.1);
      yController.setD(0);
      yController.setTolerance(2);
    } else {
      yController.setP(strafePID[0].getNumber());
      yController.setI(strafePID[1].getNumber());
      yController.setD(strafePID[2].getNumber());
      yController.setTolerance(strafePID[3].getNumber());
    }
    yController.setSetpoint(Math.sin(finalYawSetpointDegrees));
    startTime = -1;
  }

  public ChassisSpeeds update() {
    if (startTime == -1) {
      startTime = Timer.getFPGATimestamp();
    }
    yController.setP(strafePID[0].getNumber());
    yController.setI(strafePID[1].getNumber());
    yController.setD(strafePID[2].getNumber());
    yController.setTolerance(strafePID[3].getNumber());
    yController.setSetpoint(Math.sin(finalYawSetpointDegrees));
    // yController.setP(0.07);
    // yController.setTolerance(0.2);

    SmartDashboard.putBoolean("YOLO yAtSetpoint", yController.atSetpoint());

    m_BranchDetectorCamera.updateByUnreadResults();

    if (!isAlignInPlace) {
      // has drive forward and strafe
      if (!m_BranchDetectorCamera.hasTargets()) {
        return new ChassisSpeeds(0.1, 0, 0);
      } else {
        // if we have a target, strafe to align with it
        yCommand = getAlignCommand();
        return new ChassisSpeeds(driveIntoReefSpeed, yCommand, 0);
      }
    } else {
      // has strafe only
      if (!m_BranchDetectorCamera.hasTargets()) {
        // if we don't have a target, just go left
        System.out.println("no target and its all mannans fault. MANNAN YOU ARE A BRICK");
        return new ChassisSpeeds(0, 0.05 * Math.signum(yCommand), 0);
      } else {
        // if we have a target, strafe to align with it
        yCommand = getAlignCommand();
        return new ChassisSpeeds(0, yCommand, 0);
      }
    }
  }
  @Logged
  public boolean atGoal() {
    // should be stalling when driving into reef
    // return Timer.getFPGATimestamp() - startTime >= 0.5;
    // actual vx less than stall speed

    if (!isAlignInPlace) {
      // return m_Swerve.getChassisSpeeds().vxMetersPerSecond <= stallSpeedThreshold
      //     && Timer.getFPGATimestamp() - startTime >= 0.5;
      return false;
    } else {
      return yController.atSetpoint();
    }
  }
}
