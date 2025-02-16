// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Drivetrain;

import static edu.wpi.first.units.Units.*;
import static frc.robot.Constants.DrivetrainConstants.ControllerConstants.*;
import static frc.robot.Constants.FieldConstants.*;
import static frc.robot.Constants.VisionConstants.*;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import frc.lib.dashboard.TunableNumber;
import frc.lib.util.FieldPose;
import frc.lib.util.FieldPose.FieldElement;
import frc.robot.subsystems.Vision.AprilTagCamera;

/** Add your docs here. */
public class VisionController {

  private ProfiledPIDController xController;
  private ProfiledPIDController yController;
  private ProfiledPIDController thetaController;

  private final Swerve m_Swerve;

  private double lastSeenAprilTagTime;
  private AutoAlignController fallbackController;

  private TunableNumber[] xPID = {
    new TunableNumber("VisionController/Px", 0),
    new TunableNumber("VisionController/Ix", 0),
    new TunableNumber("VisionController/Dx", 0)
  };

  private TunableNumber[] yPID = {
    new TunableNumber("VisionController/Py", 0),
    new TunableNumber("VisionController/Iy", 0),
    new TunableNumber("VisionController/Dy", 0)
  };

  private TunableNumber[] thetaPID = {
    new TunableNumber("VisionController/Pt", 0),
    new TunableNumber("VisionController/It", 0),
    new TunableNumber("VisionController/Dt", 0)
  };

  public VisionController(Swerve swerve, AutoAlignController fallback) {
    xController = new ProfiledPIDController(0, 0, 0, new TrapezoidProfile.Constraints(0, 0));
    yController = new ProfiledPIDController(0, 0, 0, new TrapezoidProfile.Constraints(0, 0));
    thetaController = new ProfiledPIDController(0, 0, 0, new TrapezoidProfile.Constraints(0, 0));

    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    xController.setTolerance(toleranceTranslation);
    yController.setTolerance(toleranceTranslation);
    thetaController.setTolerance(toleranceRadians);

    xController.setConstraints(
        new TrapezoidProfile.Constraints(
            maxVelocity.in(MetersPerSecond) / Math.sqrt(2),
            maxAcceleration.in(MetersPerSecondPerSecond) / Math.sqrt(2)));
    yController.setConstraints(
        new TrapezoidProfile.Constraints(
            maxVelocity.in(MetersPerSecond) / Math.sqrt(2),
            maxAcceleration.in(MetersPerSecondPerSecond) / Math.sqrt(2)));
    thetaController.setConstraints(
        new TrapezoidProfile.Constraints(
            maxAngularVelocity.in(RadiansPerSecond),
            maxAngularAcceleration.in(RadiansPerSecondPerSecond)));

    fallbackController = fallback;

    m_Swerve = swerve;
    reset();
  }

  public void reset() {
    xController.reset(m_Swerve.getPose().getX(), m_Swerve.getChassisSpeeds().vxMetersPerSecond);
    yController.reset(m_Swerve.getPose().getY(), m_Swerve.getChassisSpeeds().vyMetersPerSecond);
    thetaController.reset(
        m_Swerve.getRotation2D().getRadians(), m_Swerve.getChassisSpeeds().omegaRadiansPerSecond);
  }

  public ChassisSpeeds update(
      AprilTagCamera primaryCamera, Pose2d currentPose, int targetTagID, FieldPose targetPose) {
    /* Update PID Controllers */
    xController.setPID(xPID[0].getNumber(), xPID[1].getNumber(), xPID[2].getNumber());
    yController.setPID(yPID[0].getNumber(), yPID[1].getNumber(), yPID[2].getNumber());
    thetaController.setPID(
        thetaPID[0].getNumber(), thetaPID[1].getNumber(), thetaPID[2].getNumber());

    if (FieldPose.fieldElementIsReef(targetPose.fieldElement)) {
      // so we don't need to set new offsets for every reef position
      // instead we just map all reef positions to REEFA
      targetPose = new FieldPose(targetPose.side, FieldElement.REEFA, targetPose.offset);
    }
    if (FieldPose.fieldElementIsHPStation(targetPose.fieldElement)) {
      // so we don't need to set new offsets for both HP stations
      // instead we just map all reef positions to TOPHPSTATION
      targetPose = new FieldPose(targetPose.side, FieldElement.TOPHPSTATION, targetPose.offset);
    }

    Translation2d targetOffset = fieldPoseOffsets.get(targetPose);

    double currentX = 0;
    double currentY = 0;
    double targetX = 0;
    double targetY = 0;
    boolean hasTarget = true;

    if (!primaryCamera.getTarget(targetTagID).isEmpty()) {
      targetX = targetOffset.getX();
      targetY = targetOffset.getY();
      currentX = primaryCamera.getX(targetTagID).getAsDouble();
      currentY = primaryCamera.getY(targetTagID).getAsDouble();
    }

    if (hasTarget) {
      lastSeenAprilTagTime = Timer.getFPGATimestamp();
    } else {
      if ((Timer.getFPGATimestamp() - lastSeenAprilTagTime) > maxTimeUntilFallbackToOdometry) {
        return fallbackController.update(m_Swerve.getPose(), fieldPoses.get(targetPose));
      } else {
        return m_Swerve.getChassisSpeeds();
      }
    }

    double xVel = xController.calculate(currentX, targetX);
    double yVel = yController.calculate(currentY, targetY);

    double radiansSetpoint = fieldPoses.get(targetPose).getRotation().getRadians();

    double thetaVel =
        -thetaController.calculate(m_Swerve.getRotation2D().getRadians(), radiansSetpoint);

    return ChassisSpeeds.fromFieldRelativeSpeeds(xVel, yVel, thetaVel, currentPose.getRotation());
  }

  public boolean atGoal() {
    return xController.atGoal() && yController.atGoal() && thetaController.atGoal();
  }
}
