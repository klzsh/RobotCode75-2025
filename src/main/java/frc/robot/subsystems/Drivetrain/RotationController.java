// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Drivetrain;

import static edu.wpi.first.units.Units.*;
import static frc.robot.Constants.DrivetrainConstants.ControllerConstants.RotationAlign.*;
import static frc.robot.Constants.DrivetrainConstants.ControllerConstants.toleranceRadians;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Strategy;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;

/** Add your docs here. */

// for implementation look here:
// https://github.com/Mechanical-Advantage/RobotCode2024/blob/main/src/main/java/org/littletonrobotics/frc2024/subsystems/drive/controllers/HeadingController.java
// TODO: Document
@Logged(name = "Rotation Controller", strategy = Strategy.OPT_IN)
public class RotationController {
  private double output;

  private final Swerve swerve;

  private PIDController controller;

  // private TunableNumber kpNumber = new TunableNumber("RotationController/Kp", kp);
  // private TunableNumber kdNumber = new TunableNumber("RotationController/Kd", kd);

  public RotationController(Swerve swerve) {
    controller =
        new PIDController(
            kp, 0, // no I term
            kd);
    controller.enableContinuousInput(-Math.PI, Math.PI);

    controller.setTolerance(toleranceRadians);
    this.swerve = swerve;

    // controller.reset(
    //     swerve.getRotation2D().getRadians(), swerve.getChassisSpeeds().omegaRadiansPerSecond);
  }

  // @Logged(name = "output", importance = Importance.INFO)
  public double getOutput() {
    return output;
  }

  public void update(Rotation2d setpoint) {
    // Update controller

    // controller.setP(kpNumber.getNumber());
    // controller.setD(kdNumber.getNumber());

    // controller.setConstraints(
    //     new TrapezoidProfile.Constraints(
    //         maxAngularVelocity.in(RadiansPerSecond),
    //         maxAngularAcceleration.in(RadiansPerSecondPerSecond)));

    controller.setSetpoint(setpoint.getRadians());
    this.output = controller.calculate(swerve.getRotation2D().getRadians(), setpoint.getRadians());
  }

  public void update(Rotation2d setpoint, double p, double d) {
    // Update controller
    controller.setPID(p, 0, d);
    update(setpoint);
  }

  public boolean atGoal() {
    return controller.atSetpoint();
  }
}
