// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Constants;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Hertz;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Seconds;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.signals.AdvancedHallSupportValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorArrangementValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Frequency;
import edu.wpi.first.units.measure.Time;

/**
 * This class is meant to house the configs for specific motors All configs from CTRE motors are
 * unit-aware, especially configs for closed loop gains timeSync can only be used on a CANivore any
 * TorqueCurrentFOC gains/control modes can only be used with Phoenix pro (HIGHLY RECCOMENDED TO
 * USE)
 */
public final class HardwareConstants {
  public static final String superstructureCANBusName = "Superstructure";
  public static final boolean TUNING_MODE = true;

  public static final class Swerve {
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
    public static final Current angleCurrentLimit = Amps.of(80);
    public static final Current angleLowerCurrentThreshold = Amps.of(40);

    public static final Current angleStatorCurrentLimit = Amps.of(120);
    public static final Current angleStatorCurrentLimitForward = Amps.of(120);
    public static final Current angleStatorCurrentLimitReverse = Amps.of(-120);
    // Seconds
    public static final Time angleCurrentThresholdTime = Seconds.of(0.50);
    public static final Time driveCurrentThresholdTime = Seconds.of(0.50);
    // amps
    public static final Current driveCurrentLimit = Amps.of(80);
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
    // TODO: tune
    public static final double angleTorqueKP = 50.0;
    public static final double angleTorqueKI = 0.0;
    public static final double angleTorqueKD = 1.0;

    /* Drive Motor PID Values */
    // TODO: tune
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
      m_DriveConfig.CurrentLimits.SupplyCurrentLimit = driveCurrentLimit.in(Amps);
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
      m_AngleConfig.CurrentLimits.SupplyCurrentLimit = angleCurrentLimit.in(Amps);
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

  public static final class EndEffector {
    public static final TalonFXSConfiguration m_CoralMotorConfig = new TalonFXSConfiguration();
    public static final TalonFXConfiguration m_AlgaeMotorConfig = new TalonFXConfiguration();
    public static final TalonFXConfiguration m_PivotConfig = new TalonFXConfiguration();
    /* CANIDS */
    // TODO: find these
    public static final int coralMotorCanID = 43;
    public static final int algaeMotorCanID = 0;
    public static final int pivotCanID = 0;
    public static final int coralBeamBreakDIO = 2;
    public static final int algaePivotEncoderPort = 0;

    public static final Rotation2d algaePivotZeroPoint = Rotation2d.fromDegrees(0);
    public static final Angle algaeEncoderOffset = Rotations.of(0);

    /* Neutral modes / inverts */
    public static final InvertedValue coralMotorInvert = InvertedValue.CounterClockwise_Positive;
    public static final InvertedValue algaeMotorInvert = InvertedValue.CounterClockwise_Positive;
    public static final InvertedValue pivotInvert = InvertedValue.CounterClockwise_Positive;

    public static final NeutralModeValue coralMotorNuetralMode = NeutralModeValue.Brake;
    public static final NeutralModeValue algaeMotorNuetralMode = NeutralModeValue.Brake;
    public static final NeutralModeValue pivotNeutralMode = NeutralModeValue.Brake;

    public static final Frequency TimeSyncFreq = Hertz.of(250);

    // coral current limiting
    public static final Current coralCurrentLimit = Amps.of(40);
    public static final Current coralCurrentLowerThreshold = Amps.of(30);

    public static final Current coralStatorCurrentLimit = Amps.of(60);
    public static final Current coralStatorCurrentLimitForward = Amps.of(60);
    public static final Current coralStatorCurrentLimitReverse = Amps.of(-60);

    public static final Time coralCurrentThresholdTime = Seconds.of(0.50);

    // algae current limiting
    public static final Current algaeCurrentLimit = Amps.of(30);
    public static final Current algaeLowerCurrentThreshold = Amps.of(15);

    public static final Current algaeStatorCurrentLimit = Amps.of(20);
    public static final Current algaeStatorCurrentLimitForward = Amps.of(20);
    public static final Current algaeStatorCurrentLimitReverse = Amps.of(-20);

    public static final Time algaeCurrentThresholdTime = Seconds.of(1);

    // pivot current limits
    public static final Current pivotCurrentLimit = Amps.of(40);
    public static final Current pivotCurrentLowerThreshold = Amps.of(30);

    public static final Current pivotStatorCurrentLimit = Amps.of(60);
    public static final Current pivotStatorCurrentLimitForward = Amps.of(60);
    public static final Current pivotStatorCurrentLimitReverse = Amps.of(-60);

    public static final Time pivotCurrentThresholdTime = Seconds.of(0.50);

    /* Torque PID */
    public static final double openLoopRamp = 0.1;
    public static final double closedLoopRamp = 0.1;

    public static final double coralVelocityKP = 2;
    public static final double coralVelocityKI = 0.0;
    public static final double coralVelocityKD = 0.0;
    public static final double coralVelocityKS = 10;

    public static final double coralPositionKP = 0.75;
    public static final double coralPositionKI = 0.0;
    public static final double coralPositionKD = 0.0;

    public static final double algaeKP = 0.5;
    public static final double algaeKI = 0.0;
    public static final double algaeKD = 0.0;

    public static final double pivotKP = 0.5;
    public static final double pivotKI = 0.0;
    public static final double pivotKD = 0.0;

    public static TalonFXSConfiguration getCoralMotorConfiguration() {

      m_CoralMotorConfig.MotorOutput.Inverted = coralMotorInvert;
      m_CoralMotorConfig.MotorOutput.NeutralMode = coralMotorNuetralMode;

      m_CoralMotorConfig.Commutation.MotorArrangement = MotorArrangementValue.Minion_JST;
      m_CoralMotorConfig.Commutation.AdvancedHallSupport = AdvancedHallSupportValue.Enabled;
      m_CoralMotorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

      /* Current Limiting */
      m_CoralMotorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
      m_CoralMotorConfig.CurrentLimits.SupplyCurrentLimit = coralCurrentLimit.in(Amps);
      m_CoralMotorConfig.CurrentLimits.SupplyCurrentLowerTime =
          coralCurrentThresholdTime.in(Seconds);
      m_CoralMotorConfig.CurrentLimits.SupplyCurrentLowerLimit =
          coralCurrentLowerThreshold.in(Amps);

      m_CoralMotorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
      m_CoralMotorConfig.CurrentLimits.StatorCurrentLimit = coralStatorCurrentLimit.in(Amps);

      /* PID Config */
      m_CoralMotorConfig.Slot0.kP = coralVelocityKP;
      m_CoralMotorConfig.Slot0.kI = coralVelocityKI;
      m_CoralMotorConfig.Slot0.kD = coralVelocityKD;
      m_CoralMotorConfig.Slot0.kS = coralVelocityKS;

      m_CoralMotorConfig.Slot1.kP = coralPositionKP;
      m_CoralMotorConfig.Slot1.kI = coralPositionKI;
      m_CoralMotorConfig.Slot1.kD = coralPositionKD;

      /* Open and Closed Loop Ramping */
      m_CoralMotorConfig.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = openLoopRamp;
      m_CoralMotorConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = openLoopRamp;

      m_CoralMotorConfig.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = closedLoopRamp;
      m_CoralMotorConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = closedLoopRamp;

      m_CoralMotorConfig.ClosedLoopRamps.TorqueClosedLoopRampPeriod = closedLoopRamp;
      m_CoralMotorConfig.MotorOutput.ControlTimesyncFreqHz = TimeSyncFreq.in(Hertz);

      return m_CoralMotorConfig;
    }

    public static TalonFXConfiguration getAlgaeMotorConfiguration() {
      m_AlgaeMotorConfig.MotorOutput.Inverted = algaeMotorInvert;
      m_AlgaeMotorConfig.MotorOutput.NeutralMode = algaeMotorNuetralMode;

      /* Current Limiting */
      m_AlgaeMotorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
      m_AlgaeMotorConfig.CurrentLimits.SupplyCurrentLimit = algaeCurrentLimit.in(Amps);
      m_AlgaeMotorConfig.CurrentLimits.SupplyCurrentLowerTime =
          algaeCurrentThresholdTime.in(Seconds);
      m_AlgaeMotorConfig.CurrentLimits.SupplyCurrentLowerLimit =
          algaeLowerCurrentThreshold.in(Amps);

      m_AlgaeMotorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
      m_AlgaeMotorConfig.CurrentLimits.StatorCurrentLimit = algaeStatorCurrentLimit.in(Amps);
      m_AlgaeMotorConfig.TorqueCurrent.PeakForwardTorqueCurrent =
          algaeStatorCurrentLimitForward.in(Amps);
      m_AlgaeMotorConfig.TorqueCurrent.PeakReverseTorqueCurrent =
          algaeStatorCurrentLimitReverse.in(Amps);

      /* PID Config */
      m_AlgaeMotorConfig.Slot0.kP = algaeKP;
      m_AlgaeMotorConfig.Slot0.kI = algaeKI;
      m_AlgaeMotorConfig.Slot0.kD = algaeKD;

      m_AlgaeMotorConfig.MotorOutput.ControlTimesyncFreqHz = TimeSyncFreq.in(Hertz);

      return m_AlgaeMotorConfig;
    }

    public static TalonFXConfiguration getPivotConfiguration() {

      m_PivotConfig.MotorOutput.Inverted = algaeMotorInvert;
      m_PivotConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

      /* Current Limiting */
      m_PivotConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
      m_PivotConfig.CurrentLimits.SupplyCurrentLimit = pivotCurrentLimit.in(Amps);
      m_PivotConfig.CurrentLimits.SupplyCurrentLowerTime = pivotCurrentThresholdTime.in(Seconds);
      m_PivotConfig.CurrentLimits.SupplyCurrentLowerLimit = pivotCurrentLowerThreshold.in(Amps);

      m_PivotConfig.CurrentLimits.StatorCurrentLimitEnable = true;
      m_PivotConfig.CurrentLimits.StatorCurrentLimit = pivotStatorCurrentLimit.in(Amps);
      m_PivotConfig.TorqueCurrent.PeakForwardTorqueCurrent =
          pivotStatorCurrentLimitForward.in(Amps);
      m_PivotConfig.TorqueCurrent.PeakReverseTorqueCurrent =
          pivotStatorCurrentLimitReverse.in(Amps);

      /* PID Config */
      m_PivotConfig.Slot0.kP = pivotKP;
      m_PivotConfig.Slot0.kI = pivotKI;
      m_PivotConfig.Slot0.kD = pivotKD;

      m_PivotConfig.MotorOutput.ControlTimesyncFreqHz = TimeSyncFreq.in(Hertz);

      m_PivotConfig.ClosedLoopGeneral.ContinuousWrap = true;
      m_PivotConfig.Feedback.RotorToSensorRatio = 25.0 / 1.0; // constants?

      return m_PivotConfig;
    }
  }

  public static final class Elevator {

    // takes 0.25 seconds to go from 0-100% current output
    public static final Time closedLoopRamp = Seconds.of(0.25);

    public static final Current statorCurrentLimit = Amps.of(60);
    public static final Current supplyCurrentLimit = Amps.of(40);
    // set current limit to 30 amps if supply current limit is exceeded for more than 0.5 seconds
    public static final Current supplyCurrentLowerLimit = Amps.of(30);
    public static final Time supplyCurrentLowerTime = Seconds.of(0.5);

    public static final Angle forwardLimit = Rotations.of(26);
    public static final Angle reverseLimit = Rotations.of(0);

    public static final Frequency timeSyncFreq = Hertz.of(250);

    public static final double kA = 0.5; // current per unit of acceleration
    public static final double kG = 21; // current to overcome gravity
    public static final double kS = 17; // current to overcome static friction
    public static final double kV = 0.12; // current per unit of requested velocity
    public static final double kP = 20;
    public static final double kI = 0;
    public static final double kD = 5;

    public static final Current torqueForwardCurrentLimit = Amps.of(100);
    public static final Current torqueReverseCurrentLimit = Amps.of(100);

    public static TalonFXConfiguration getElevatorMotorConfig() {
      TalonFXConfiguration m_ElevatorMotorConfig = new TalonFXConfiguration();

      m_ElevatorMotorConfig.ClosedLoopRamps.TorqueClosedLoopRampPeriod = closedLoopRamp.in(Seconds);

      m_ElevatorMotorConfig.CurrentLimits.StatorCurrentLimit = statorCurrentLimit.in(Amps);
      m_ElevatorMotorConfig.CurrentLimits.StatorCurrentLimitEnable = true;

      m_ElevatorMotorConfig.CurrentLimits.SupplyCurrentLimit = supplyCurrentLimit.in(Amps);
      m_ElevatorMotorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
      m_ElevatorMotorConfig.CurrentLimits.SupplyCurrentLowerLimit =
          supplyCurrentLowerLimit.in(Amps);
      m_ElevatorMotorConfig.CurrentLimits.SupplyCurrentLowerTime =
          supplyCurrentLowerTime.in(Seconds);

      m_ElevatorMotorConfig.MotorOutput.ControlTimesyncFreqHz = timeSyncFreq.in(Hertz);
      m_ElevatorMotorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
      m_ElevatorMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

      m_ElevatorMotorConfig.Slot0.GravityType = GravityTypeValue.Elevator_Static;
      m_ElevatorMotorConfig.Slot0.StaticFeedforwardSign =
          StaticFeedforwardSignValue.UseVelocitySign;
      m_ElevatorMotorConfig.Slot0.kA = kA; // tune third
      m_ElevatorMotorConfig.Slot0.kG = kG; // tune first
      m_ElevatorMotorConfig.Slot0.kS = kS; // tune second
      m_ElevatorMotorConfig.Slot0.kV = kV; // tune third
      m_ElevatorMotorConfig.Slot0.kP = kP; // tune fourth
      m_ElevatorMotorConfig.Slot0.kI = kI; // tune only if needed
      m_ElevatorMotorConfig.Slot0.kD = kD; // tune fifth

      m_ElevatorMotorConfig.TorqueCurrent.PeakForwardTorqueCurrent =
          torqueForwardCurrentLimit.in(Amps);
      m_ElevatorMotorConfig.TorqueCurrent.PeakReverseTorqueCurrent =
          torqueReverseCurrentLimit.in(Amps);

      m_ElevatorMotorConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
      m_ElevatorMotorConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold =
          forwardLimit.in(Rotations);
      m_ElevatorMotorConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
      m_ElevatorMotorConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold =
          reverseLimit.in(Rotations);

      return m_ElevatorMotorConfig;
    }
  }
}
