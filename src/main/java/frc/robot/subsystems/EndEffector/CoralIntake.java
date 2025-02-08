// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.EndEffector;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.Constants.EndEffectorConstants.*;
import static frc.robot.Constants.HardwareConstants.EndEffector.*;
import static frc.robot.Constants.HardwareConstants.superstructureCANBusName;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFXS;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Importance;
import edu.wpi.first.epilogue.Logged.Strategy;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.dashboard.TunableNumber;

@Logged(name = "Coral Intake", strategy = Strategy.OPT_IN)
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

  @Logged(name = "Coral Motor", importance = Importance.DEBUG)
  private TalonFXS m_CoralMotor;

  private DigitalInput m_CoralBeamBreak;
  // TODO: add tunable numbers for PID configs, velocity speed
  private Slot0Configs VelocityPIDConfig = new Slot0Configs();
  private Slot1Configs PositionPIDConfig = new Slot1Configs();
  private final TunableNumber coralIntakeVelocityKp;
  private final TunableNumber coralIntakeVelocityKd;
  private final TunableNumber coralIntakeVelocityKs;

  private final TunableNumber coralIntakePositionKp;
  private final TunableNumber coralIntakePositionKI;
  private final TunableNumber coralIntakePositionKd;

  private final TunableNumber coralScoreVelocity;
  private final TunableNumber coralIntakeVelocity;
  private final TunableNumber coralPositionToMove;

  private final VoltageOut m_CharacterizationRequest;
  private final VelocityVoltage m_VelocityRequest;
  private final PositionVoltage m_PositionRequest;

  /** Creates a new CoralIntake. */
  public CoralIntake() {
    m_CoralMotor = new TalonFXS(coralMotorCanID, superstructureCANBusName);
    m_CoralIntakeState = CoralStates.DEFAULT;
    m_CoralBeamBreak = new DigitalInput(coralBeamBreakDIO);

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

    VelocityPIDConfig.withKS(coralVelocityKS).withKP(coralVelocityKP).withKD(coralVelocityKD);

    PositionPIDConfig.withKP(coralPositionKP).withKI(coralPositionKI).withKD(coralPositionKD);

    coralIntakeVelocityKp = new TunableNumber("Coral Intake/Velocity kP", coralVelocityKP);
    coralIntakeVelocityKd = new TunableNumber("Coral Intake/Velocity kD", coralVelocityKD);
    coralIntakeVelocityKs = new TunableNumber("Coral Intake/Velocity kS", coralVelocityKS);

    coralIntakePositionKp = new TunableNumber("Coral Intake/Position kP", coralPositionKP);
    coralIntakePositionKd = new TunableNumber("Coral Intake/Position kI", coralPositionKI);
    coralIntakePositionKI = new TunableNumber("Coral Intake/Position kD", coralPositionKD);

    coralScoreVelocity =
        new TunableNumber("Coral Intake/Score Velocity", coralScoreSpeed.in(RotationsPerSecond));
    coralIntakeVelocity =
        new TunableNumber("Coral Intake/Intake Velocity", coralIntakeSpeed.in(RotationsPerSecond));
    coralPositionToMove =
        new TunableNumber("Coral Intake/Position To Move", coralRotationsAfterIntake.in(Rotations));
  }

  public void setState(CoralStates state) {
    m_CoralIntakeState = state;
  }

  @Logged(name = "Coral State", importance = Importance.CRITICAL)
  public CoralStates getState() {
    return m_CoralIntakeState;
  }

  @Logged(name = "Beam Break", importance = Importance.CRITICAL)
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
        < positionDeadband;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (coralIntakeVelocityKp.getNumber() != VelocityPIDConfig.kP
        || coralIntakeVelocityKd.getNumber() != VelocityPIDConfig.kD
        || coralIntakeVelocityKs.getNumber() != VelocityPIDConfig.kS) {
      VelocityPIDConfig.kP = coralIntakeVelocityKp.getNumber();
      VelocityPIDConfig.kD = coralIntakeVelocityKd.getNumber();
      VelocityPIDConfig.kS = coralIntakeVelocityKs.getNumber();

      m_CoralMotor.getConfigurator().apply(VelocityPIDConfig);
    }

    if (coralIntakePositionKp.getNumber() != PositionPIDConfig.kP
        || coralIntakePositionKd.getNumber() != PositionPIDConfig.kD
        || coralIntakePositionKI.getNumber() != PositionPIDConfig.kI) {
      PositionPIDConfig.kP = coralIntakePositionKp.getNumber();
      PositionPIDConfig.kD = coralIntakePositionKd.getNumber();
      PositionPIDConfig.kI = coralIntakePositionKI.getNumber();

      m_CoralMotor.getConfigurator().apply(PositionPIDConfig);
    }

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
        m_CoralMotor.setControl(m_VelocityRequest.withVelocity(coralScoreVelocity.getNumber()).withSlot(0));
      }
      case INTAKING -> {
        m_CoralMotor.setControl(m_VelocityRequest.withVelocity(coralIntakeVelocity.getNumber()).withSlot(0));
      }
      case POSITIONING -> {
        m_CoralMotor.setControl(
            m_PositionRequest.withPosition(coralPositionToMove.getNumber() * coralMotorGearRatio).withSlot(1));
      }
      case DEFAULT -> {
        m_CoralMotor.setControl(m_CharacterizationRequest.withOutput(0));
      }
    }
  }
}
