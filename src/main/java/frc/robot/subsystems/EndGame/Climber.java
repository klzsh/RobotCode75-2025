// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.EndGame;

import static edu.wpi.first.units.Units.*;
import static frc.robot.Constants.ClimberConstants.*;
import static frc.robot.Constants.HardwareConstants.*;
import static frc.robot.Constants.HardwareConstants.Climber.*;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.MotionMagicExpoTorqueCurrentFOC;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Importance;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.dashboard.TunableNumber;
import frc.robot.Constants.ClimberConstants;

public class Climber extends SubsystemBase {

  public static enum ClimberPositions {
    DEFAULT,
    CLIMB
  }

  @Logged(name = "Climber Motor 1", importance = Importance.DEBUG)
  private final TalonFX m_ClimberMotor1;

  @Logged(name = "Climber Motor 2", importance = Importance.DEBUG)
  private final TalonFX m_ClimberMotor2;

  @Logged(name = "Climber State", importance = Importance.CRITICAL)
  private ClimberPositions m_ClimberState = ClimberPositions.DEFAULT;

  private Slot0Configs PIDConfig = new Slot0Configs();
  private final TunableNumber climberKp;
  private final TunableNumber climberKd;
  private final TunableNumber climberKg;
  private final TunableNumber climberKs;
  private final TunableNumber climberKa;
  private final TunableNumber climberKv;

  private MotionMagicConfigs MMConfig = new MotionMagicConfigs();
  private final TunableNumber climberMMCruiseVelocity;
  private final TunableNumber climberMMKv;
  private final TunableNumber climberMMKa;

  private final MotionMagicExpoTorqueCurrentFOC m_PositionRequest;

  private final TorqueCurrentFOC m_TestRequest;
  private final DigitalInput m_LimitSwitch;

  public Climber() {
    m_ClimberMotor1 = new TalonFX(climberMotor1CANID, superstructureCANBusName);
    m_ClimberMotor2 = new TalonFX(climberMotor2CANID, superstructureCANBusName);

    climberMMCruiseVelocity = new TunableNumber("Climber/Cruise Velocity", MMcruiseVelocity);
    climberMMKv = new TunableNumber("Climber/MM kV", MMkV);
    climberMMKa = new TunableNumber("Climber/MM kA", MMkA);

    m_PositionRequest = new MotionMagicExpoTorqueCurrentFOC(0);
    m_TestRequest = new TorqueCurrentFOC(Amps.of(0));

    PIDConfig.withKA(kA)
        .withKS(kS)
        .withKV(kV)
        .withKG(kG)
        .withKP(kP)
        .withKD(kD)
        .withGravityType(GravityTypeValue.Arm_Cosine)
        .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseVelocitySign);

    climberKp = new TunableNumber("Climber/kP", kP);
    climberKd = new TunableNumber("Climber/kD", kD);
    climberKg = new TunableNumber("Climber/kG", kG);
    climberKs = new TunableNumber("Climber/kS", kS);
    climberKa = new TunableNumber("Climber/kA", kA);
    climberKv = new TunableNumber("Climber/kV", kV);

    m_ClimberMotor1.getConfigurator().apply(getClimberMotorConfig());
    m_ClimberMotor2.getConfigurator().apply(getClimberMotorConfig());

    m_LimitSwitch = new DigitalInput(limitPort);
  }

  public void setState(ClimberPositions state) {
    m_ClimberState = state;
  }

  public ClimberPositions getState() {
    return m_ClimberState;
  }

  private void setPositionRequest(Angle position) {
    m_ClimberMotor1.setControl(
        m_PositionRequest.withPosition(position).withLimitForwardMotion(getLimitSwitch()));
    m_ClimberMotor2.setControl(
        m_PositionRequest.withPosition(position).withLimitForwardMotion(getLimitSwitch()));
  }

  @Logged(name = "Climber Position", importance = Importance.CRITICAL)
  public Angle getPosition() {
    return Rotations.of(
        (m_ClimberMotor1.getPosition().getValue().in(Rotations)
                + m_ClimberMotor2.getPosition().getValue().in(Rotations))
            / 2);
  }

  public boolean atPosition() {
    return Math.abs(getPosition().in(Rotations) - climbPosition.in(Rotations)) < climbDeadband;
  }

  @Logged(name = "Climber Limit", importance = Importance.DEBUG)
  public boolean getLimitSwitch() {
    return !m_LimitSwitch.get();
  }
  public void runPosition(double current){
    m_ClimberMotor1.setControl(m_TestRequest.withOutput(current));
    m_ClimberMotor2.setControl(m_TestRequest.withOutput(current));
  }

  @Override
  public void periodic() {
    if (climberKp.getNumber() != PIDConfig.kP
        || climberKd.getNumber() != PIDConfig.kD
        || climberKs.getNumber() != PIDConfig.kS
        || climberKg.getNumber() != PIDConfig.kG
        || climberKa.getNumber() != PIDConfig.kA
        || climberKv.getNumber() != PIDConfig.kV) {
      PIDConfig.kP = climberKp.getNumber();
      PIDConfig.kD = climberKd.getNumber();
      PIDConfig.kS = climberKs.getNumber();
      PIDConfig.kG = climberKg.getNumber();
      PIDConfig.kA = climberKa.getNumber();
      PIDConfig.kV = climberKv.getNumber();

      m_ClimberMotor1.getConfigurator().apply(PIDConfig);
      m_ClimberMotor2.getConfigurator().apply(PIDConfig);
    }
    if (climberMMCruiseVelocity.getNumber() != MMConfig.MotionMagicCruiseVelocity
        || climberMMKv.getNumber() != MMConfig.MotionMagicExpo_kV
        || climberMMKa.getNumber() != MMConfig.MotionMagicExpo_kA) {

      MMConfig.MotionMagicCruiseVelocity = climberMMCruiseVelocity.getNumber();
      MMConfig.MotionMagicExpo_kV = climberMMKv.getNumber();
      MMConfig.MotionMagicExpo_kA = climberMMKa.getNumber();

      m_ClimberMotor1.getConfigurator().apply(MMConfig);
      m_ClimberMotor2.getConfigurator().apply(MMConfig);
    }

    switch (m_ClimberState) {
      case DEFAULT -> {
        setPositionRequest(Rotations.of(0));
      }
      case CLIMB -> {
        if (getLimitSwitch()) {
          setPositionRequest(getPosition());
        } else {
          setPositionRequest(ClimberConstants.climbPosition);
        }
      }
    }
  }
}
