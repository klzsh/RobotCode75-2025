// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Constants;

import static edu.wpi.first.units.Units.Hertz;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.units.measure.Frequency;

/** This class is meant to house the configs for specific motors */
public final class HardwareConstants {

  public static final class EndEffector {
    public static final TalonFXConfiguration m_CoralMotorConfig = new TalonFXConfiguration();
    public static final TalonFXConfiguration m_AlgaeMotorConfig = new TalonFXConfiguration();

    /* CANIDS */
    public static final int coralMotorCanID = 0;
    public static final int algaeMotorCanID = 0;
    public static final int coralBeamBreakCanDIO = 0;

    /* Neutral modes / inverts */
    public static final InvertedValue coralMotorInvert = InvertedValue.Clockwise_Positive;
    public static final InvertedValue algaeMotorInvert = InvertedValue.Clockwise_Positive;
    public static final NeutralModeValue coralMotorNuetralMode = NeutralModeValue.Brake;
    public static final NeutralModeValue algaeMotorNuetralMode = NeutralModeValue.Brake;

    public static final Frequency TimeSyncFreq = Hertz.of(250);

    
    /* End Effector Current Limiting (Amps) */
    public static final int coralCurrentLimit = 80;
    public static final int coralCurrentLowerThreshold = 40;

    public static final boolean coralStatorCurrnetLimitEnable = true;
    public static final int coralStatorCurrentLimit = 20;
    public static final int coralStatorCurrentLimitForward = 20;
    public static final int coralStatorCurrentLimitReverse = -20;
    
    // Seconds
    public static final double coralCurrentThresholdTime = 0.50;
    public static final boolean coralEnableCurrentLimit = true;

    // amps
    public static final int algaeCurrentLimit = 80;
    public static final int algaeLowerCurrentThreshold = 40;

    public static final boolean algaeStatorCurrnetLimitEnable = true;
    public static final int algaeStatorCurrentLimit = 20;
    public static final int algaeStatorCurrentLimitForward = 20;
    public static final int algaeStatorCurrentLimitReverse = -20;

    // seconds
    public static final double algaeCurrentThresholdTime = 0.50;
    public static final boolean algaeEnableCurrentLimit = true;

    /* Torque PID */
    public static final double openLoopRamp = 0.1;
    public static final double closedLoopRamp = 0.1;

    public static final double coralTorqueKP = 2.0;
    public static final double coralTorqueKI = 0.0;
    public static final double coralTorqueKD = 0.0;
    
    public static final double algaeTorqueKP = 2.0;
    public static final double algaeTorqueKI = 0.0;
    public static final double algaeTorqueKD = 0.0;

    public static TalonFXConfiguration getCoralMotorConfiguration() {

      m_CoralMotorConfig.MotorOutput.Inverted = coralMotorInvert;
      m_CoralMotorConfig.MotorOutput.NeutralMode = coralMotorNuetralMode;

      /* Current Limiting */
      m_CoralMotorConfig.CurrentLimits.SupplyCurrentLimitEnable = coralEnableCurrentLimit;
      m_CoralMotorConfig.CurrentLimits.SupplyCurrentLimit = coralCurrentLimit;
      m_CoralMotorConfig.CurrentLimits.SupplyCurrentLowerTime = coralCurrentThresholdTime;
      m_CoralMotorConfig.CurrentLimits.SupplyCurrentLowerLimit = coralCurrentLowerThreshold;

      m_CoralMotorConfig.CurrentLimits.StatorCurrentLimitEnable = coralStatorCurrnetLimitEnable;
      m_CoralMotorConfig.CurrentLimits.StatorCurrentLimit = coralStatorCurrentLimit;
      m_CoralMotorConfig.TorqueCurrent.PeakForwardTorqueCurrent = coralStatorCurrentLimitForward;
      m_CoralMotorConfig.TorqueCurrent.PeakReverseTorqueCurrent = coralStatorCurrentLimitReverse;

      /* PID Config */
      m_CoralMotorConfig.Slot0.kP = coralTorqueKP;
      m_CoralMotorConfig.Slot0.kI = coralTorqueKI;
      m_CoralMotorConfig.Slot0.kD = coralTorqueKD;

      /* Open and Closed Loop Ramping */
      m_CoralMotorConfig.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = openLoopRamp;
      m_CoralMotorConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = openLoopRamp;

      m_CoralMotorConfig.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = closedLoopRamp;
      m_CoralMotorConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = closedLoopRamp;
      // TODO: see if nessesary
      m_CoralMotorConfig.ClosedLoopRamps.TorqueClosedLoopRampPeriod = closedLoopRamp;
      m_CoralMotorConfig.MotorOutput.ControlTimesyncFreqHz = TimeSyncFreq.in(Hertz);

      return m_CoralMotorConfig;
    }

    public static TalonFXConfiguration getAlgaeMotorConfiguration() {
      m_AlgaeMotorConfig.MotorOutput.Inverted = algaeMotorInvert;
      m_AlgaeMotorConfig.MotorOutput.NeutralMode = algaeMotorNuetralMode;

      /* Current Limiting */
      m_AlgaeMotorConfig.CurrentLimits.SupplyCurrentLimitEnable = algaeEnableCurrentLimit;
      m_AlgaeMotorConfig.CurrentLimits.SupplyCurrentLimit = algaeCurrentLimit;
      m_AlgaeMotorConfig.CurrentLimits.SupplyCurrentLowerTime = algaeCurrentThresholdTime;
      m_AlgaeMotorConfig.CurrentLimits.SupplyCurrentLowerLimit = algaeLowerCurrentThreshold;

      m_AlgaeMotorConfig.CurrentLimits.StatorCurrentLimitEnable = algaeStatorCurrnetLimitEnable;
      m_AlgaeMotorConfig.CurrentLimits.StatorCurrentLimit = algaeStatorCurrentLimit;
      m_AlgaeMotorConfig.TorqueCurrent.PeakForwardTorqueCurrent = algaeStatorCurrentLimitForward;
      m_AlgaeMotorConfig.TorqueCurrent.PeakReverseTorqueCurrent = algaeStatorCurrentLimitReverse;

      /* PID Config */
      m_AlgaeMotorConfig.Slot0.kP = algaeTorqueKP;
      m_AlgaeMotorConfig.Slot0.kI = algaeTorqueKI;
      m_AlgaeMotorConfig.Slot0.kD = algaeTorqueKD;

      m_AlgaeMotorConfig.MotorOutput.ControlTimesyncFreqHz = TimeSyncFreq.in(Hertz);

      return m_AlgaeMotorConfig;
    }
  }

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
}
