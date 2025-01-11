// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Constants;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;

/** This class is meant to house the configs for specific motors */
public final class HardwareConstants {

  public static final String superstructureCANBusName = "Superstructure";

  public static final class Swerve {
    public static final TalonFXConfiguration m_DriveConfig = new TalonFXConfiguration();
    public static final TalonFXConfiguration m_AngleConfig = new TalonFXConfiguration();
    public static final CANcoderConfiguration m_EncoderConfig = new CANcoderConfiguration();
    /* define actual constants */

    /* Motor Inverts */
    public static final InvertedValue angleMotorInvert = InvertedValue.CounterClockwise_Positive;
    public static final InvertedValue driveMotorInvert = InvertedValue.CounterClockwise_Positive;

    /* Angle Encoder Invert */
    public static final SensorDirectionValue cancoderInvert =
        SensorDirectionValue.CounterClockwise_Positive;

    /* Swerve Current Limiting (Amps) */
    public static final int angleCurrentLimit = 80;
    public static final int angleLowerCurrentThreshold = 40;

    public static final boolean angleStatorCurrnetLimitEnable = true;
    public static final int angleStatorCurrentLimit = 120;
    public static final int angleStatorCurrentLimitForward = 120;
    public static final int angleStatorCurrentLimitReverse = -120;
    // Seconds
    public static final double angleCurrentThresholdTime = 0.50;
    public static final double driveCurrentThresholdTime = 0.50;
    // amps
    public static final int driveCurrentLimit = 80;
    public static final int driveCurrentLowerThreshold = 40;

    public static final boolean driveStatorCurrnetLimitEnable = true;
    public static final int driveStatorCurrentLimit = 80;
    public static final int driveStatorCurrentLimitForward = 80;
    public static final int driveStatorCurrentLimitReverse = -80;
    // seconds
    public static final boolean angleEnableCurrentLimit = true;
    public static final boolean driveEnableCurrentLimit = true;

    /*
     * These values are used by the drive falcon to ramp in open loop and closed
     * loop driving.
     * We found a small open loop ramp (0.25 sec) helps with tread wear, tipping,
     * etc
     */
    public static final double openLoopRamp = 0.25;
    public static final double closedLoopRamp = 0.25;

    public static final double angleTorqueKP = 50.0;
    public static final double angleTorqueKI = 0.0;
    public static final double angleTorqueKD = 1.0;

    /* Drive Motor PID Values */

    // TORQUE PID Values
    public static final double driveTorqueKP = 2;
    public static final double driveTorqueKI = 0.0;
    public static final double driveTorqueKD = 0.0;
    public static final double driveTorqueKS = 10; // 19.449

    /* Neutral Modes */
    public static final NeutralModeValue angleNeutralMode = NeutralModeValue.Coast;
    public static final NeutralModeValue driveNeutralMode = NeutralModeValue.Brake;

    public static TalonFXConfiguration getDriveConfiguration() {

      m_DriveConfig.MotorOutput.Inverted = driveMotorInvert;
      m_DriveConfig.MotorOutput.NeutralMode = driveNeutralMode;

      /* Current Limiting */
      m_DriveConfig.CurrentLimits.SupplyCurrentLimitEnable = driveEnableCurrentLimit;
      m_DriveConfig.CurrentLimits.SupplyCurrentLimit = driveCurrentLimit;
      m_DriveConfig.CurrentLimits.SupplyCurrentLowerTime = driveCurrentThresholdTime;
      m_DriveConfig.CurrentLimits.SupplyCurrentLowerLimit = driveCurrentLowerThreshold;

      m_DriveConfig.CurrentLimits.StatorCurrentLimitEnable = driveStatorCurrnetLimitEnable;
      m_DriveConfig.CurrentLimits.StatorCurrentLimit = driveStatorCurrentLimit;
      m_DriveConfig.TorqueCurrent.PeakForwardTorqueCurrent = driveStatorCurrentLimitForward;
      m_DriveConfig.TorqueCurrent.PeakReverseTorqueCurrent = driveStatorCurrentLimitReverse;

      /* PID Config */
      m_DriveConfig.Slot0.kP = driveTorqueKP;
      m_DriveConfig.Slot0.kI = driveTorqueKI;
      m_DriveConfig.Slot0.kD = driveTorqueKD;
      m_DriveConfig.Slot0.kS = driveTorqueKS;

      /* Open and Closed Loop Ramping */
      m_DriveConfig.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = openLoopRamp;
      m_DriveConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = openLoopRamp;

      m_DriveConfig.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = closedLoopRamp;
      m_DriveConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = closedLoopRamp;
      // TODO: see if nessesary
      m_DriveConfig.ClosedLoopRamps.TorqueClosedLoopRampPeriod = closedLoopRamp;

      return m_DriveConfig;
    }

    public static TalonFXConfiguration getAngleConfiguration() {
      m_AngleConfig.MotorOutput.Inverted = angleMotorInvert;
      m_AngleConfig.MotorOutput.NeutralMode = angleNeutralMode;

      /* Current Limiting */
      m_AngleConfig.CurrentLimits.SupplyCurrentLimitEnable = angleEnableCurrentLimit;
      m_AngleConfig.CurrentLimits.SupplyCurrentLimit = angleCurrentLimit;
      m_AngleConfig.CurrentLimits.SupplyCurrentLowerTime = angleCurrentThresholdTime;
      m_AngleConfig.CurrentLimits.SupplyCurrentLowerLimit = angleLowerCurrentThreshold;

      m_AngleConfig.CurrentLimits.StatorCurrentLimitEnable = angleStatorCurrnetLimitEnable;
      m_AngleConfig.CurrentLimits.StatorCurrentLimit = angleStatorCurrentLimit;
      m_AngleConfig.TorqueCurrent.PeakForwardTorqueCurrent = angleStatorCurrentLimitForward;
      m_AngleConfig.TorqueCurrent.PeakReverseTorqueCurrent = angleStatorCurrentLimitReverse;

      /* PID Config */
      m_AngleConfig.Slot0.kP = angleTorqueKP;
      m_AngleConfig.Slot0.kI = angleTorqueKI;
      m_AngleConfig.Slot0.kD = angleTorqueKD;

      // TODO: try this out
      // m_AngleConfig.ClosedLoopGeneral.ContinuousWrap = true;

      return m_AngleConfig;
    }

    public static CANcoderConfiguration getEncoderConfiguration() {
      m_EncoderConfig.MagnetSensor.SensorDirection = cancoderInvert;
      return m_EncoderConfig;
    }
  }

  public static final class Elevator {

    // takes 0.25 seconds to go from 0-100% current output
    public static final double closedLoopRamp = 0.25;

    // reasonable starting points TODO: make sure these are sane
    public static final double statorCurrentLimit = 60;
    public static final double supplyCurrentLimit = 40;
    // set current limit to 30 amps if supply current limit is exceeded for more than 0.5 seconds
    public static final double supplyCurrentLowerLimit = 30;
    public static final double supplyCurrentLowerTime = 0.5;

    // TODO: find these
    public static final double mmAcceleration = 0;
    public static final double mmCruiseVelocity = 0;
    public static final double mmExpoKa = 0;
    public static final double mmExpoKv = 0;
    public static final double mmJerk = 0;

    public static final double timeSyncFreq = 250;

    // TODO: find these
    public static final double kA = 0;
    public static final double kG = 0;
    public static final double kS = 0;
    public static final double kV = 0;
    public static final double kP = 0;
    public static final double kI = 0;
    public static final double kD = 0;

    // TODO: slowly ramp these up
    public static final double torqueForwardCurrentLimit = 20;
    public static final double torqueReverseCurrentLimit = 20;

    public static TalonFXConfiguration getElevatorMotorConfig(boolean inverted) {
      TalonFXConfiguration m_ElevatorMotorConfig = new TalonFXConfiguration();

      m_ElevatorMotorConfig.ClosedLoopRamps.TorqueClosedLoopRampPeriod = closedLoopRamp;

      m_ElevatorMotorConfig.CurrentLimits.StatorCurrentLimit = statorCurrentLimit;
      m_ElevatorMotorConfig.CurrentLimits.StatorCurrentLimitEnable = true;

      m_ElevatorMotorConfig.CurrentLimits.SupplyCurrentLimit = supplyCurrentLimit;
      m_ElevatorMotorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
      m_ElevatorMotorConfig.CurrentLimits.SupplyCurrentLowerLimit = supplyCurrentLowerLimit;
      m_ElevatorMotorConfig.CurrentLimits.SupplyCurrentLowerTime = supplyCurrentLowerTime;

      // TODO: decide on which motion magic config to use
      m_ElevatorMotorConfig.MotionMagic.MotionMagicAcceleration = mmAcceleration;
      m_ElevatorMotorConfig.MotionMagic.MotionMagicCruiseVelocity = mmCruiseVelocity;
      m_ElevatorMotorConfig.MotionMagic.MotionMagicExpo_kA = mmExpoKa;
      m_ElevatorMotorConfig.MotionMagic.MotionMagicExpo_kV = mmExpoKv;
      m_ElevatorMotorConfig.MotionMagic.MotionMagicJerk = mmJerk;

      m_ElevatorMotorConfig.MotorOutput.ControlTimesyncFreqHz = timeSyncFreq;
      m_ElevatorMotorConfig.MotorOutput.Inverted =
          inverted ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
      m_ElevatorMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

      m_ElevatorMotorConfig.Slot0.GravityType = GravityTypeValue.Elevator_Static;
      m_ElevatorMotorConfig.Slot0.StaticFeedforwardSign =
          StaticFeedforwardSignValue.UseVelocitySign;
      m_ElevatorMotorConfig.Slot0.kA = kA;
      m_ElevatorMotorConfig.Slot0.kG = kG;
      m_ElevatorMotorConfig.Slot0.kS = kS;
      m_ElevatorMotorConfig.Slot0.kV = kV;
      m_ElevatorMotorConfig.Slot0.kP = kP;
      m_ElevatorMotorConfig.Slot0.kI = kI;
      m_ElevatorMotorConfig.Slot0.kD = kD;

      m_ElevatorMotorConfig.TorqueCurrent.PeakForwardTorqueCurrent = torqueForwardCurrentLimit;
      m_ElevatorMotorConfig.TorqueCurrent.PeakReverseTorqueCurrent = torqueReverseCurrentLimit;

      return m_ElevatorMotorConfig;
    }
  }
}
