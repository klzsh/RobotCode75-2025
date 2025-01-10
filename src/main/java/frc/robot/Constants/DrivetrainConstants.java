// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Constants;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import frc.lib.util.SwerveModuleConstants;

/** Add your docs here. */
public final class DrivetrainConstants {
  // TODO: make sure this is correct
  public static final boolean invertGyro = false; // Always ensure Gyro is CCW+ CW-

  public static final int kPigeonID = 18;

  public static final boolean USE_ANGLE_TORQUE = true;
  public static final boolean USE_DRIVE_TORQUE = true;

  /* Drivetrain Constants */
  // TODO: Find these
  public static final Distance trackWidth = Inches.of(22.5);
  public static final Distance wheelBase = Inches.of(22.5);
  // distance between the center of the wheel to the center of the robot
  public static final Distance kCenterToWheel =
      Meters.of(Math.hypot(trackWidth.in(Meters), wheelBase.in(Meters)) / 2);
  public static final Distance wheelDiameter = Inches.of(2);
  public static final Distance wheelCircumference = Meters.of(wheelDiameter.in(Meters) * Math.PI);

  public static final class ControllerConstants {
    public static final double kp = 0.08;
    public static final double kd = 0.03;
    public static final double maxVelocityMultiplier = 0.8;
    public static final double maxAccelerationMultiplier = 0.8;
    public static final double loopPeriodSeconds = 0.02;
    public static final double toleranceRadians = .15;
  }

  /*
   * Swerve Kinematics
   * No need to ever change this unless you are not doing a traditional
   * rectangular/square 4 module swerve
   */
  public static final SwerveDriveKinematics swerveKinematics =
      new SwerveDriveKinematics(
          new Translation2d(
              DrivetrainConstants.wheelBase.in(Meters) / 2.0,
              DrivetrainConstants.trackWidth.in(Meters) / 2.0),
          new Translation2d(
              DrivetrainConstants.wheelBase.in(Meters) / 2.0,
              -DrivetrainConstants.trackWidth.in(Meters) / 2.0),
          new Translation2d(
              -DrivetrainConstants.wheelBase.in(Meters) / 2.0,
              DrivetrainConstants.trackWidth.in(Meters) / 2.0),
          new Translation2d(
              -DrivetrainConstants.wheelBase.in(Meters) / 2.0,
              -DrivetrainConstants.trackWidth.in(Meters) / 2.0));


  /* Module Gear Ratios */
  // ratio of motor turns to mechanism turns
  public static final double driveGearRatio = 6.75;
  public static final double angleGearRatio = 12.8;

  /** Meters per Second */
  public static final LinearVelocity maxSpeed = MetersPerSecond.of(4.49);

  /** Radians per Second */
  public static final AngularVelocity maxAngularVelocity =
      RadiansPerSecond.of(
          maxSpeed.in(MetersPerSecond)
              / Math.hypot(trackWidth.in(Meters) / 2.0, wheelBase.in(Meters) / 2.0));

  /** Radians per Second per Second */
  public static final AngularAcceleration maxAngularAcceleration =
      RadiansPerSecondPerSecond.of(
          maxSpeed.in(MetersPerSecond)
              / Math.hypot(trackWidth.in(Meters) / 2.0, wheelBase.in(Meters) / 2.0));

  /** Drivetrain CANBus name */
  public static final String driveBusName = "Drivetrain";

  /** Radius of robot from center to wheel */

  /* Module Specific Constants */
  // ! module offsets are configured. Only change if a serious deviation is seen,
  // as well as in
  // ! each comp
  /* Front Left Module - Module 0 */
  public static final class FrontLeft {
    public static final int driveMotorID = 14;
    public static final int angleMotorID = 24;
    public static final int canCoderID = 34;
    public static final Rotation2d angleOffset = Rotation2d.fromDegrees(75.14);

    public static final SwerveModuleConstants constants =
        new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
  }

  /* Front Right Module - Module 1 */
  public static final class FrontRight {
    public static final int driveMotorID = 11;
    public static final int angleMotorID = 21;
    public static final int canCoderID = 31;
    public static final Rotation2d angleOffset = Rotation2d.fromDegrees(73.38);

    public static final SwerveModuleConstants constants =
        new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
  }

  /* Back Left Module - Module 2 */
  public static final class BackLeft {
    public static final int driveMotorID = 13;
    public static final int angleMotorID = 23;
    public static final int canCoderID = 33;
    public static final Rotation2d angleOffset = Rotation2d.fromDegrees(127.44);

    public static final SwerveModuleConstants constants =
        new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
  }

  /* Back Right Module - Module 3 */
  public static final class BackRight {
    public static final int driveMotorID = 12;
    public static final int angleMotorID = 22;
    public static final int canCoderID = 32;
    public static final Rotation2d angleOffset = Rotation2d.fromDegrees(-125.85);

    public static final SwerveModuleConstants constants =
        new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
  }
}
