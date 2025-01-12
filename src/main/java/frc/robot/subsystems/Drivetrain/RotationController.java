// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Drivetrain;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.Constants.DrivetrainConstants;

/** Add your docs here. */

// for implementation look here:
// https://github.com/Mechanical-Advantage/RobotCode2024/blob/main/src/main/java/org/littletonrobotics/frc2024/subsystems/drive/controllers/HeadingController.java
// TODO: Document
public class RotationController {
  private double output;

  private final Swerve swerve;

  private ProfiledPIDController controller;

  public RotationController(Swerve swerve) {
    controller =
        new ProfiledPIDController(
            DrivetrainConstants.ControllerConstants.kp,
            0,
            DrivetrainConstants.ControllerConstants.kd,
            new TrapezoidProfile.Constraints(0.0, 0.0),
            DrivetrainConstants.ControllerConstants.loopPeriodSeconds);
    controller.enableContinuousInput(-Math.PI, Math.PI);

    controller.setTolerance(DrivetrainConstants.ControllerConstants.toleranceRadians);
    this.swerve = swerve;

    controller.reset(
        swerve.getRotation2D().getRadians(), swerve.getChassisSpeeds().omegaRadiansPerSecond);
  }

  public double getOutput() {
    return output;
  }

  public void update(Rotation2d setpoint) {
    // Update controller

    controller.setConstraints(
        new TrapezoidProfile.Constraints(
            DrivetrainConstants.maxAngularVelocity.in(RadiansPerSecond),
            DrivetrainConstants.maxAngularAcceleration.in(RadiansPerSecondPerSecond)));

    this.output = -controller.calculate(swerve.getRotation2D().getRadians(), setpoint.getRadians());
  }

  public void update(Rotation2d setpoint, double p, double d) {
    // Update controller
    controller.setPID(p, 0, d);
    update(setpoint);
  }

  public boolean atGoal() {
    return controller.atGoal();
  }
}
