// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Constants;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.units.measure.*;
import frc.lib.util.SwerveModuleConstants;

/** Swerve drive constants */
public final class DrivetrainConstants {
  /** Drivetrain CANBus name */
  public static final String driveBusName = "Drivetrain";

  public static final boolean invertGyro = false; // Always ensure Gyro is CCW+ CW-

  public static final int kPigeonID = 18;

  /* Drivetrain Constants */
  public static final Distance trackLength = Inches.of(22.75);
  public static final Distance trackWidth = Inches.of(22.75);

  public static final Distance wheelDiameter = Inches.of(4);
  public static final Distance wheelCircumference = Meters.of(wheelDiameter.in(Meters) * Math.PI);

  public static final class ControllerConstants {
    public static final double toleranceRadians = 1.5;
    public static final double toleranceTranslation = .01;

    public static final class RotationAlign {

      public static final double kp = 0.05;
      public static final double kd = 0;
      public static final double maxVelocityMultiplier = 0.8;
      public static final double maxAccelerationMultiplier = 0.8;
      public static final double loopPeriodSeconds = 0.02;
    }

    public static final class OdometryAlign {
      public static final double xP =  4;
      public static final double xI = 0.0;
      public static final double xD = 0.0;

      public static final double tP = 6;
      public static final double tI = 0.0;
      public static final double tD = 0.0;
    }

    public static final class VisionAlign {
      public static final double xP = 0.0;
      public static final double xI = 0.0;
      public static final double xD = 0.0;

      public static final double yP = 0.7;
      public static final double yI = 0.0;
      public static final double yD = 0.0;
    }

    public static final LinearVelocity maxVelocity = MetersPerSecond.of(1);
    public static final LinearAcceleration maxAcceleration = MetersPerSecondPerSecond.of(1);

    /** Radians per Second */
    public static final AngularVelocity maxAngularVelocity =
        RadiansPerSecond.of(
            maxSpeed.in(MetersPerSecond)
                / Math.hypot(trackWidth.in(Meters) / 2.0, trackLength.in(Meters) / 2.0));

    /** Radians per Second per Second */
    public static final AngularAcceleration maxAngularAcceleration =
        RadiansPerSecondPerSecond.of(
            maxSpeed.in(MetersPerSecond)
                / Math.hypot(trackWidth.in(Meters) / 2.0, trackLength.in(Meters) / 2.0));

    public static final AngularVelocity maxAngularVelocityAuto = maxAngularVelocity.div(2);
    public static final AngularAcceleration maxAngularAccelerationAuto =
        maxAngularAcceleration.div(2);
  }

  /*
   * Swerve Kinematics
   * No need to ever change this unless you are not doing a traditional
   * rectangular/square 4 module swerve
   */
  public static final SwerveDriveKinematics swerveKinematics =
      new SwerveDriveKinematics(
          new Translation2d(
              DrivetrainConstants.trackWidth.in(Meters) / 2.0,
              DrivetrainConstants.trackLength.in(Meters) / 2.0),
          new Translation2d(
              DrivetrainConstants.trackWidth.in(Meters) / 2.0,
              -DrivetrainConstants.trackLength.in(Meters) / 2.0),
          new Translation2d(
              -DrivetrainConstants.trackWidth.in(Meters) / 2.0,
              DrivetrainConstants.trackLength.in(Meters) / 2.0),
          new Translation2d(
              -DrivetrainConstants.trackWidth.in(Meters) / 2.0,
              -DrivetrainConstants.trackLength.in(Meters) / 2.0));

  /* Module Gear Ratios */
  // ratio of motor turns to mechanism turns
  public static final double driveGearRatio = 6.75; // L2
  public static final double angleGearRatio = 150.0 / 7.0; // ~21:1 ratio

  /** Meters per Second */
  public static final LinearVelocity maxSpeed = MetersPerSecond.of(3.5);

  public static final LinearAcceleration maxAcceleration = MetersPerSecondPerSecond.of(3);

  /** Radians per Second */
  public static final AngularVelocity maxAngularVelocity =
      RadiansPerSecond.of(
          maxSpeed.in(MetersPerSecond)
              / Math.hypot(trackLength.in(Meters) / 2.0, trackWidth.in(Meters) / 2.0));

  /** Radians per Second per Second */
  public static final AngularAcceleration maxAngularAcceleration =
      RadiansPerSecondPerSecond.of(
          maxSpeed.in(MetersPerSecond)
              / Math.hypot(trackLength.in(Meters) / 2.0, trackWidth.in(Meters) / 2.0));

  /* Module Specific Constants */
  // ! Only change if a serious deviation is seen, as well as in each comp
  /* Front Left Module - Module 0 */
  public static final class FrontLeft {
    public static final int driveMotorID = 14;
    public static final int angleMotorID = 24;
    public static final int canCoderID = 34;
    public static final Rotation2d angleOffset = Rotation2d.fromDegrees(-47.373);

    public static final SwerveModuleConstants constants =
        new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
  }

  /* Front Right Module - Module 1 */
  public static final class FrontRight {
    public static final int driveMotorID = 11;
    public static final int angleMotorID = 21;
    public static final int canCoderID = 31;
    public static final Rotation2d angleOffset = Rotation2d.fromDegrees(113.906);

    public static final SwerveModuleConstants constants =
        new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
  }

  /* Back Left Module - Module 2 */
  public static final class BackLeft {
    public static final int driveMotorID = 13;
    public static final int angleMotorID = 23;
    public static final int canCoderID = 33;
    public static final Rotation2d angleOffset = Rotation2d.fromDegrees(113.818);

    public static final SwerveModuleConstants constants =
        new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
  }

  /* Back Right Module - Module 3 */
  public static final class BackRight {
    public static final int driveMotorID = 12;
    public static final int angleMotorID = 22;
    public static final int canCoderID = 32;
    public static final Rotation2d angleOffset = Rotation2d.fromDegrees(-147.744);

    public static final SwerveModuleConstants constants =
        new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
  }

  public static final class MotorConfigs {
    public static final TalonFXConfiguration m_DriveConfig = new TalonFXConfiguration();
    public static final TalonFXConfiguration m_AngleConfig = new TalonFXConfiguration();
    public static final CANcoderConfiguration m_EncoderConfig = new CANcoderConfiguration();
    /* define actual constants */

    /* Motor Inverts */
    public static final InvertedValue angleMotorInvert = InvertedValue.Clockwise_Positive;
    public static final InvertedValue driveMotorInvert = InvertedValue.Clockwise_Positive;

    /* Angle Encoder Invert */
    public static final SensorDirectionValue cancoderInvert =
        SensorDirectionValue.CounterClockwise_Positive;

    /* Swerve Current Limiting (Amps) */
    public static final Current angleSupplyCurrentLimit = Amps.of(80);
    public static final Current angleLowerCurrentThreshold = Amps.of(40);

    public static final Current angleStatorCurrentLimit = Amps.of(60);
    public static final Current angleStatorCurrentLimitForward = Amps.of(60);
    public static final Current angleStatorCurrentLimitReverse = Amps.of(-60);
    // Seconds
    public static final Time angleCurrentThresholdTime = Seconds.of(0.50);
    public static final Time driveCurrentThresholdTime = Seconds.of(0.50);
    // amps
    public static final Current driveSupplyCurrentLimit = Amps.of(80);
    public static final Current driveCurrentLowerThreshold = Amps.of(40);

    public static final Current driveStatorCurrentLimit = Amps.of(80);
    public static final Current driveStatorCurrentLimitForward = Amps.of(80);
    public static final Current driveStatorCurrentLimitReverse = Amps.of(-80);

    /*
     * These values are used by the drive motor to ramp in open loop and closed
     * loop driving.
     * We found a small open loop ramp (0.25 sec) helps with tread wear, tipping,
     * etc
     */
    public static final Time openLoopRamp = Seconds.of(0.25);
    public static final Time closedLoopRamp = Seconds.of(0.5);
    public static final double angleTorqueKP = 50.0;
    public static final double angleTorqueKI = 0.0;
    public static final double angleTorqueKD = 1.0;

    /* Drive Motor PID Values */
    public static final double driveTorqueKP = 1.93;
    public static final double driveTorqueKI = 0.0;
    public static final double driveTorqueKD = 0.0;
    public static final double driveTorqueKS = 10; // static feedforward

    /* Neutral Modes */
    public static final NeutralModeValue angleNeutralMode = NeutralModeValue.Coast;
    public static final NeutralModeValue driveNeutralMode = NeutralModeValue.Brake;

    public static final Frequency timeSyncFreq = Hertz.of(250);

    public static TalonFXConfiguration getDriveConfiguration() {

      m_DriveConfig.MotorOutput.Inverted = driveMotorInvert;
      m_DriveConfig.MotorOutput.NeutralMode = driveNeutralMode;

      /* Current Limiting */
      m_DriveConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
      m_DriveConfig.CurrentLimits.SupplyCurrentLimit = driveSupplyCurrentLimit.in(Amps);
      m_DriveConfig.CurrentLimits.SupplyCurrentLowerTime = driveCurrentThresholdTime.in(Seconds);
      m_DriveConfig.CurrentLimits.SupplyCurrentLowerLimit = driveCurrentLowerThreshold.in(Amps);

      m_DriveConfig.CurrentLimits.StatorCurrentLimitEnable = true;
      m_DriveConfig.CurrentLimits.StatorCurrentLimit = driveStatorCurrentLimit.in(Amps);
      m_DriveConfig.TorqueCurrent.PeakForwardTorqueCurrent =
          driveStatorCurrentLimitForward.in(Amps);
      m_DriveConfig.TorqueCurrent.PeakReverseTorqueCurrent =
          driveStatorCurrentLimitReverse.in(Amps);

      /* PID Config */
      m_DriveConfig.Slot0.kP = driveTorqueKP;
      m_DriveConfig.Slot0.kI = driveTorqueKI;
      m_DriveConfig.Slot0.kD = driveTorqueKD;
      m_DriveConfig.Slot0.kS = driveTorqueKS;

      /* Open and Closed Loop Ramping */
      m_DriveConfig.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = openLoopRamp.in(Seconds);
      m_DriveConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = openLoopRamp.in(Seconds);

      m_DriveConfig.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = closedLoopRamp.in(Seconds);
      m_DriveConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = closedLoopRamp.in(Seconds);

      m_DriveConfig.ClosedLoopRamps.TorqueClosedLoopRampPeriod = closedLoopRamp.in(Seconds);
      m_DriveConfig.MotorOutput.ControlTimesyncFreqHz = timeSyncFreq.in(Hertz);

      return m_DriveConfig;
    }

    public static TalonFXConfiguration getAngleConfiguration() {
      m_AngleConfig.MotorOutput.Inverted = angleMotorInvert;
      m_AngleConfig.MotorOutput.NeutralMode = angleNeutralMode;

      /* Current Limiting */
      m_AngleConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
      m_AngleConfig.CurrentLimits.SupplyCurrentLimit = angleSupplyCurrentLimit.in(Amps);
      m_AngleConfig.CurrentLimits.SupplyCurrentLowerTime = angleCurrentThresholdTime.in(Seconds);
      m_AngleConfig.CurrentLimits.SupplyCurrentLowerLimit = angleLowerCurrentThreshold.in(Amps);

      m_AngleConfig.CurrentLimits.StatorCurrentLimitEnable = true;
      m_AngleConfig.CurrentLimits.StatorCurrentLimit = angleStatorCurrentLimit.in(Amps);
      m_AngleConfig.TorqueCurrent.PeakForwardTorqueCurrent =
          angleStatorCurrentLimitForward.in(Amps);
      m_AngleConfig.TorqueCurrent.PeakReverseTorqueCurrent =
          angleStatorCurrentLimitReverse.in(Amps);

      /* PID Config */
      m_AngleConfig.Slot0.kP = angleTorqueKP;
      m_AngleConfig.Slot0.kI = angleTorqueKI;
      m_AngleConfig.Slot0.kD = angleTorqueKD;

      m_AngleConfig.MotorOutput.ControlTimesyncFreqHz = timeSyncFreq.in(Hertz);

      return m_AngleConfig;
    }

    public static CANcoderConfiguration getEncoderConfiguration() {
      m_EncoderConfig.MagnetSensor.SensorDirection = cancoderInvert;
      return m_EncoderConfig;
    }
  }
}
