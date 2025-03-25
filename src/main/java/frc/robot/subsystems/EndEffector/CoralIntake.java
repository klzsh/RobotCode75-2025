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
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Importance;
import edu.wpi.first.epilogue.Logged.Strategy;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.dashboard.TunableNumber;

@Logged(name = "Coral Intake", strategy = Strategy.OPT_IN, importance = Importance.CRITICAL)
public class CoralIntake extends SubsystemBase {
  public static enum CoralStates {
    // no intaking state because coral sits on top of the intake wheels
    HASGAMEPIECE,
    SCORING,
    INTAKING,
    POSITIONING,
    REVERSING,
    DEFAULT // default is when the intake is doing nothing
  }

  private static CoralStates m_CoralIntakeState;

  // @Logged(name = "Coral Motor", importance = Importance.DEBUG)
  private TalonFXS m_CoralMotor;

  private DigitalInput m_CoralBeamBreak;
  private boolean m_isL1 = false;

  private final VoltageOut m_CharacterizationRequest;
  private final VelocityVoltage m_VelocityRequest;
  private final PositionVoltage m_PositionRequest;

  private final TunableNumber scoreSpeed;
  private final TunableNumber rotationsAfterIntake;

  // private final TunableNumber coralVelocitykP;
  // private final TunableNumber coralVelocitykI;
  // private final TunableNumber coralVelocitykD;
  // private final TunableNumber coralVelocitykS;

  // private final TunableNumber coralPositionkP;
  // private final TunableNumber coralPositionkI;
  // private final TunableNumber coralPositionkD;

  // private Slot0Configs velocityConfig = new Slot0Configs();
  // private Slot1Configs positionConfig = new Slot1Configs();

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

    scoreSpeed =
        new TunableNumber("Coral Intake/Score Speed", coralScoreSpeed.in(RotationsPerSecond));
    rotationsAfterIntake =
        new TunableNumber(
            "Coral Intake/Rotations After Intake", coralRotationsAfterIntake.in(Rotations));

    // coralVelocitykP = new TunableNumber("Coral Intake/Velocity Kp", coralVelocityKP);
    // coralVelocitykI = new TunableNumber("Coral Intake/Velocity Ki", coralVelocityKI);
    // coralVelocitykD = new TunableNumber("Coral Intake/Velocity Kd", coralVelocityKD);
    // coralVelocitykS = new TunableNumber("Coral Intake/Velocity Ks", coralVelocityKS);

    // coralPositionkP = new TunableNumber("Coral Intake/Position Kp", coralPositionKP);
    // coralPositionkI = new TunableNumber("Coral Intake/Position Ki", coralPositionKI);
    // coralPositionkD = new TunableNumber("Coral Intake/Position Kd", coralPositionKD);

  }

  public void setState(CoralStates state) {
    m_CoralIntakeState = state;
  }

  public void setL1(boolean isL1) {
    m_isL1 = isL1;
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
    return m_CoralMotor.getPosition(true).getValue().in(Rotations)
        > coralRotationsAfterIntake.in(Rotations);
  }

  @Logged(name = "Velocity", importance = Importance.CRITICAL)
  public double getVelocity() {
    return m_CoralMotor.getVelocity(true).getValue().in(RotationsPerSecond);
  }

  @Logged(name = "Position", importance = Importance.CRITICAL)
  public double getPosition() {
    return m_CoralMotor.getPosition(true).getValue().in(Rotations);
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("Has Coral", getBeamBreak());

    //     if (coralVelocitykP.getNumber() != velocityConfig.kP
    //     || coralVelocitykI.getNumber() != velocityConfig.kI
    //     || coralVelocitykD.getNumber() != velocityConfig.kD
    //     || coralVelocitykS.getNumber() != velocityConfig.kS) {
    //       velocityConfig.kP = coralVelocitykP.getNumber();
    //       velocityConfig.kD = coralVelocitykD.getNumber();
    //       velocityConfig.kS = coralVelocitykS.getNumber();

    //   m_CoralMotor.getConfigurator().apply(velocityConfig);
    // }

    // if (coralPositionkP.getNumber() != positionConfig.kP
    // || coralPositionkI.getNumber() != positionConfig.kI
    // || coralPositionkD.getNumber() != positionConfig.kD) {
    //   positionConfig.kP = coralPositionkP.getNumber();
    //   positionConfig.kI = coralPositionkI.getNumber();
    //   positionConfig.kD = coralPositionkD.getNumber();

    // m_CoralMotor.getConfigurator().apply(positionConfig);
    // }

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

    double scoreMultiplier = 1;
    if (DriverStation.isAutonomous()) {
      scoreMultiplier = 2;
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
        if (m_isL1) {
          m_CoralMotor.setControl(m_VelocityRequest.withVelocity(coralScoreSpeedL1).withSlot(0));
        } else {
          // m_CoralMotor.setControl(m_VelocityRequest.withVelocity(coralScoreSpeed).withSlot(0));
          m_CoralMotor.setControl(
              m_VelocityRequest.withVelocity(scoreSpeed.getNumber() * scoreMultiplier).withSlot(0));
        }
      }
      case INTAKING -> {
        m_CoralMotor.setControl(m_VelocityRequest.withVelocity(coralIntakeSpeed.times(scoreMultiplier)).withSlot(0));
      }
      case POSITIONING -> {
        // m_CoralMotor.setControl(
        //     m_PositionRequest
        //         .withPosition(coralRotationsAfterIntake.in(Rotations) * coralMotorGearRatio)
        //         .withSlot(1));
        m_CoralMotor.setControl(
            m_PositionRequest
                .withPosition(coralRotationsAfterIntake.in(Rotations) * coralMotorGearRatio)
                .withSlot(1));
      }
      case REVERSING -> {
        m_CoralMotor.setControl(m_VelocityRequest.withVelocity(coralReverseSpeed).withSlot(0));
      }
      case DEFAULT -> {
        m_CoralMotor.setControl(m_CharacterizationRequest.withOutput(0));
      }
    }
  }
}
