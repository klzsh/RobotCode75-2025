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
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.subsystems.Drivetrain.Swerve;

@Logged(name = "YOLO Controller", importance = Importance.DEBUG, strategy = Strategy.OPT_IN)
public class YoloController {
  // private final TunableNumber[] strafePID = {
  //   new TunableNumber("YOLO Align/P", 0.035),
  //   new TunableNumber("YOLO Align/I", 0),
  //   new TunableNumber("YOLO Align/D", 0.002),
  //   new TunableNumber("YOLO Align/Tolerance", 0.25)
  // };

  private final Swerve m_Swerve;
  private final ObjectDetetectorCamera m_BranchDetectorCamera;
  private boolean isAlignInPlace;
  private ChassisSpeeds desiredSpeeds;

  // private final PIDController rotationController; // add if needed, poss just pass through a
  // heading or do this as a seperate command
  // private final PIDController xController;
  private final PIDController yController;
  // private double xCommand;
  @Logged private double yCommand;

  // for smoothify
  // private double targetYaw = -100;

  // private final TunableNumber inPlaceYP;
  // private final TunableNumber inPlaceYD;

  // private final TunableNumber driveIntoReefSpeed =
  //     new TunableNumber("YOLO Align/driveIntoReefSpeed", .25);

  private final double finalYawSetpointDegrees = -2.2;
  private final double driveIntoReefSpeed = .75;
  private final double stallSpeedThreshold = .05;
  double startTime = -1;

  /** Creates a new YoloController. */
  public YoloController(Swerve swerve, ObjectDetetectorCamera branchCam) {
    m_Swerve = swerve;
    m_BranchDetectorCamera = branchCam;
    desiredSpeeds = new ChassisSpeeds();
    yController =
        new PIDController(
            DrivetrainConstants.ControllerConstants.VisionAlign.xP,
            DrivetrainConstants.ControllerConstants.VisionAlign.xI,
            DrivetrainConstants.ControllerConstants.VisionAlign.xD);

    yController.setTolerance(.2);
    yController.setSetpoint(finalYawSetpointDegrees);
  }

  public double getAlignCommand() {
    double targetYaw = m_BranchDetectorCamera.getTargetYaw(0).getAsDouble();
    yCommand = yController.calculate(targetYaw) * -Math.signum(targetYaw);
    return yCommand;
  }

  public void reset(boolean alignInPlace) {
    isAlignInPlace = alignInPlace;
    if (isAlignInPlace) {
      yController.setP(.035);
      yController.setD(0.002);
      yController.setTolerance(.2);
      // yController.setP(strafePID[0].getNumber());
      // yController.setI(strafePID[1].getNumber());
      // yController.setD(strafePID[2].getNumber());
      // yController.setTolerance(strafePID[3].getNumber());
    } else {
      // yController.setP(strafePID[0].getNumber());
      // yController.setI(strafePID[1].getNumber());
      // yController.setD(strafePID[2].getNumber());
      // yController.setTolerance(strafePID[3].getNumber());
      yController.setP(0.08);
      yController.setD(0.005);
      yController.setTolerance(.025);
    }
    yController.setSetpoint(finalYawSetpointDegrees);
    desiredSpeeds = new ChassisSpeeds(0, 0, 0);
    startTime = -1;

    // for smoothify
    // targetYaw = -100;
  }

  public ChassisSpeeds update() {
    if (startTime == -1) {
      startTime = Timer.getFPGATimestamp();
    }

    // yController.setP(strafePID[0].getNumber());
    // yController.setI(strafePID[1].getNumber());
    // yController.setD(strafePID[2].getNumber());
    // yController.setTolerance(strafePID[3].getNumber());
    // yController.setSetpoint(finalYawSetpointDegrees);
    // yController.setP(0.07);
    // yController.setTolerance(0.2);

    m_BranchDetectorCamera.updateByUnreadResults();

    if (!isAlignInPlace) {
      if (!m_BranchDetectorCamera.hasTargets()) {
        if (!desiredSpeeds.equals(new ChassisSpeeds(0, 0, 0))) { // not fresh command
          desiredSpeeds.vyMetersPerSecond =
              desiredSpeeds.vyMetersPerSecond * .75; // should move in same direction but slower
        } else {
          desiredSpeeds = new ChassisSpeeds(driveIntoReefSpeed, 0, 0);
        }
      } else {
        
        // potential smoothify code?
        // if (targetYaw != -100 && Math.abs(targetYaw -
        // m_BranchDetectorCamera.getTargetYaw(0).getAsDouble()) > 10) {
        //   targetYaw = targetYaw * .9;
        // }
        // else {
        //   targetYaw = m_BranchDetectorCamera.getTargetYaw(0).getAsDouble();
        // }

        double targetYaw = m_BranchDetectorCamera.getTargetYaw(0).getAsDouble();

        yCommand = yController.calculate(targetYaw);

        desiredSpeeds.vxMetersPerSecond = driveIntoReefSpeed;
        desiredSpeeds.vyMetersPerSecond = yCommand;
      }
    } else {
      if (!m_BranchDetectorCamera.hasTargets()) {
        desiredSpeeds = //! Took out Y command creep speed
            new ChassisSpeeds(0, 0.0, 0); // scoot toward direction of last seen target and shi
      } else {
        double targetYaw = m_BranchDetectorCamera.getTargetYaw(0).getAsDouble();

        yCommand = yController.calculate(targetYaw);
        desiredSpeeds.vxMetersPerSecond = 0; // can't push into reef while trying to align
        desiredSpeeds.vyMetersPerSecond = yCommand;
      }
    }
    return desiredSpeeds;
  }

  @Logged
  public boolean atGoal() {
    // should be stalling when driving into reef
    // return Timer.getFPGATimestamp() - startTime >= 0.5;
    // actual vx less than stall speed

    if (!isAlignInPlace) {
      return false;
      // return m_Swerve.getChassisSpeeds().vxMetersPerSecond <= stallSpeedThreshold
      //     && (Timer.getFPGATimestamp() - startTime) >= 0.2;
    } else {
      return yController.atSetpoint();
    }
  }
}
