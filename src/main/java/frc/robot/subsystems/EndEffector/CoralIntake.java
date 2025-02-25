// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.EndEffector;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.Constants.EndEffectorConstants.*;
import static frc.robot.Constants.EndEffectorConstants.MotorConfigs.*;
import static frc.robot.Constants.RobotConstants.superstructureCANBusName;

import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFXS;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

// @Logged(name = "Coral Intake", strategy = Strategy.OPT_IN)
public class CoralIntake extends SubsystemBase {
  public static enum CoralStates {
    // no intaking state because coral sits on top of the intake wheels
    HASGAMEPIECE,
    SCORING,
    INTAKING,
    POSITIONING,
    DEFAULT // default is when the intake is doing nothing
  }

  private static CoralStates m_CoralIntakeState;

  // @Logged(name = "Coral Motor", importance = Importance.DEBUG)
  private TalonFXS m_CoralMotor;

  private DigitalInput m_CoralBeamBreak;

  private final VoltageOut m_CharacterizationRequest;
  private final VelocityVoltage m_VelocityRequest;
  private final PositionVoltage m_PositionRequest;

  /** Creates a new CoralIntake. */
  public CoralIntake() {
    m_CoralMotor = new TalonFXS(coralMotorCanID, superstructureCANBusName);
    m_CoralIntakeState = CoralStates.DEFAULT;
    m_CoralBeamBreak = new DigitalInput(coralBeamBreakPort);

    m_CharacterizationRequest = new VoltageOut(Volts.of(0));
    m_VelocityRequest = new VelocityVoltage(RotationsPerSecond.of(0));
    m_PositionRequest = new PositionVoltage(Rotations.of(0));

    m_CharacterizationRequest.EnableFOC = true;
    m_CharacterizationRequest.UpdateFreqHz = 0;
    m_CharacterizationRequest.UseTimesync = true;

    m_VelocityRequest.EnableFOC = true;
    m_VelocityRequest.UpdateFreqHz = 0;
    m_VelocityRequest.UseTimesync = true;

    m_PositionRequest.EnableFOC = true;
    m_PositionRequest.UpdateFreqHz = 0;
    m_PositionRequest.UseTimesync = true;

    m_CoralMotor.getConfigurator().apply(getCoralMotorConfiguration());
  }

  public void setState(CoralStates state) {
    m_CoralIntakeState = state;
  }

  // @Logged(name = "Coral State", importance = Importance.CRITICAL)
  public CoralStates getState() {
    return m_CoralIntakeState;
  }

  // @Logged(name = "Beam Break", importance = Importance.CRITICAL)
  public boolean getBeamBreak() {
    return !m_CoralBeamBreak.get();
  }

  public void setCoralPosition(Angle angle) {
    m_CoralMotor.setPosition(angle);
  }

  private boolean atPosition() {
    return Math.abs(
            m_CoralMotor.getPosition().getValue().in(Rotations)
                - coralRotationsAfterIntake.in(Rotations))
        < coralPositionDeadband;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    if (getBeamBreak() && m_CoralIntakeState == CoralStates.INTAKING) {
      m_CoralIntakeState = CoralStates.POSITIONING;
      m_CoralMotor.setPosition(Rotations.of(0));
    }

    if (getBeamBreak() && m_CoralIntakeState == CoralStates.POSITIONING && atPosition()) {
      m_CoralIntakeState = CoralStates.HASGAMEPIECE;
    }

    if (!getBeamBreak() && m_CoralIntakeState != CoralStates.INTAKING) {
      m_CoralIntakeState = CoralStates.DEFAULT;
    }

    switch (m_CoralIntakeState) {
      case HASGAMEPIECE -> {
        // motors do not move, beam break is broken
        // state does not change the motor output, just states that there is a gamepeice
        // in the robot
        // this state should never be set by an external command
        m_CoralMotor.setControl(m_CharacterizationRequest.withOutput(0));
      }
      case SCORING -> {
        m_CoralMotor.setControl(m_VelocityRequest.withVelocity(coralScoreSpeed).withSlot(0));
      }
      case INTAKING -> {
        m_CoralMotor.setControl(m_VelocityRequest.withVelocity(coralIntakeSpeed).withSlot(0));
      }
      case POSITIONING -> {
        m_CoralMotor.setControl(
            m_PositionRequest
                .withPosition(coralRotationsAfterIntake.in(Rotations) * coralMotorGearRatio)
                .withSlot(1));
      }
      case DEFAULT -> {
        m_CoralMotor.setControl(m_CharacterizationRequest.withOutput(0));
      }
    }
  }
}
