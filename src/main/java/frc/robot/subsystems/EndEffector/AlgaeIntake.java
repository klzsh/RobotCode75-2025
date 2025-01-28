// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.EndEffector;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import static frc.robot.Constants.EndEffectorConstants.*;
import static frc.robot.Constants.HardwareConstants.Elevator.*;
import static frc.robot.Constants.HardwareConstants.Elevator.kS;
import static frc.robot.Constants.HardwareConstants.EndEffector.*;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;


import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.dashboard.TunableNumber;

public class AlgaeIntake extends SubsystemBase {
  public static enum AlgaeStates {
    NONE,
    INTAKING,
    HASGAMEPIECE,
    OUTAKING
  }

  public static enum PivotState {
    RETRACTED,
    GROUNDINTAKE,
    DEAGLAEFY,
    NONE
  }

  // TODO: add tunable numbers for PIDS, velocity, Position, CURRENT LIMITS
  private AlgaeStates m_AlgaeIntakeState;
  private PivotState m_PivotState;
  private TalonFX m_AlgaeMotor;
  private TalonFX m_AlgaePivot;

  private Slot0Configs IntakePIDConfig = new Slot0Configs();
  private final TunableNumber algaeIntakeKp;
  private final TunableNumber algaeIntakeKd;
  private final TunableNumber algaeIntakeKg;
  private final TunableNumber algaeIntakeKs;

  private Slot0Configs PivotPIDConfig = new Slot0Configs();
  private final TunableNumber algaePivotKp;
  private final TunableNumber algaePivotKd;
  private final TunableNumber algaePivotKs;
  private final TunableNumber algaePivotKg;

  private final VelocityTorqueCurrentFOC algaeRequest =
      new VelocityTorqueCurrentFOC(RotationsPerSecond.of(0));
  private final TorqueCurrentFOC currentOut = new TorqueCurrentFOC(Amps.of(0));
  private final PositionTorqueCurrentFOC pivotRequest =
      new PositionTorqueCurrentFOC(Rotations.of(0));

  /** Creates a new AlgaeIntake. */
  public AlgaeIntake() {
    m_AlgaeMotor = new TalonFX(algaeMotorCanID);
    m_AlgaeMotor.getConfigurator().apply(getAlgaeMotorConfiguration());
    m_AlgaeIntakeState = AlgaeStates.NONE;

    m_AlgaePivot = new TalonFX(pivotCanID);
    m_AlgaePivot.getConfigurator().apply(getpivotConfiguration());
    m_PivotState = PivotState.NONE;

    algaeRequest.UpdateFreqHz = 0;
    algaeRequest.UseTimesync = true;

    currentOut.UpdateFreqHz = 0;
    currentOut.UseTimesync = true;

    pivotRequest.UpdateFreqHz = 0;
    pivotRequest.UseTimesync = true;

    IntakePIDConfig.withKA(kA)
        .withKS(kS)
        .withKV(kV)
        .withKG(kG)
        .withKP(kP)
        .withKD(kD)
        .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseVelocitySign);

    algaeIntakeKp = new TunableNumber("Algae Intake/kP", kP);
    algaeIntakeKd = new TunableNumber("Algae Intake/kD", kD);
    algaeIntakeKg = new TunableNumber("Algae Intake/kG", kG);
    algaeIntakeKs = new TunableNumber("Algae Intake/kS", kS);

    PivotPIDConfig.withKA(kA)
        .withKS(kS)
        .withKV(kV)
        .withKG(kG)
        .withKP(kP)
        .withKD(kD)
        .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseVelocitySign);

    algaePivotKp = new TunableNumber("Algae Pivot/kP", kP);
    algaePivotKd = new TunableNumber("Algae Pivot/kD", kD);
    algaePivotKg = new TunableNumber("Algae Pivot/kG", kG);
    algaePivotKs = new TunableNumber("Algae Pivot/kS", kS);
  }

  public void setAlgaeState(AlgaeStates state) {
    if (m_AlgaeIntakeState == AlgaeStates.HASGAMEPIECE && state != AlgaeStates.OUTAKING) {
      m_AlgaeIntakeState = AlgaeStates.HASGAMEPIECE;
    } else {
      m_AlgaeIntakeState = state;
    }
  }

  public void setPivotState(PivotState state) {
    m_PivotState = state;
  }

  public AlgaeStates getAlgaeState() {
    return m_AlgaeIntakeState;
  }

  public PivotState getPivotState() {
    return m_PivotState;
  }


  @Override
  public void periodic() {
    if (algaeIntakeKp.getNumber() != IntakePIDConfig.kP
        || algaeIntakeKd.getNumber() != IntakePIDConfig.kD
        || algaeIntakeKs.getNumber() != IntakePIDConfig.kS
        || algaeIntakeKg.getNumber() != IntakePIDConfig.kG) {
      IntakePIDConfig.kP = algaeIntakeKp.getNumber();
      IntakePIDConfig.kD = algaeIntakeKd.getNumber();
      IntakePIDConfig.kS = algaeIntakeKs.getNumber();
      IntakePIDConfig.kG = algaeIntakeKg.getNumber();

      m_AlgaeMotor.getConfigurator().apply(IntakePIDConfig);
    }

    if (algaePivotKp.getNumber() != PivotPIDConfig.kP
        || algaePivotKd.getNumber() != PivotPIDConfig.kD
        || algaePivotKs.getNumber() != PivotPIDConfig.kS
        || algaePivotKg.getNumber() != IntakePIDConfig.kG) {
      PivotPIDConfig.kP = algaePivotKp.getNumber();
      PivotPIDConfig.kD = algaePivotKd.getNumber();
      PivotPIDConfig.kS = algaePivotKs.getNumber();
      PivotPIDConfig.kG = algaePivotKg.getNumber();

      m_AlgaePivot.getConfigurator().apply(PivotPIDConfig);
    }

    // This method will be called once per scheduler run
    if (m_AlgaeMotor.getFault_StatorCurrLimit().getValue()) {
      m_AlgaeIntakeState = AlgaeStates.HASGAMEPIECE;
    }

    switch (m_AlgaeIntakeState) {
      case INTAKING -> {
        m_AlgaeMotor.setControl(algaeRequest.withVelocity(algaeIntakeSpeed));
      }
      case HASGAMEPIECE -> {
        // set the velocity control to a very low value (like 1-2 rps) to hold the algae in. The
        // motor will stall trying to get to the desired speed, and that will help hold the ball in;
        m_AlgaeMotor.setControl(algaeRequest.withVelocity(algaeHoldSpeed));
      }
      case OUTAKING -> {
        m_AlgaeMotor.setControl(algaeRequest.withVelocity(algaeOutakeSpeed));
      }
      case NONE -> {
        // set a NONE state for when there is no algae and we are not intaking anything
        m_AlgaeMotor.setControl(currentOut.withOutput(Amps.of(0)));
      }
        // no default case because all states are accounted for
    }
    switch (m_PivotState) {
      case RETRACTED -> {
        m_AlgaePivot.setControl(pivotRequest.withPosition(pivotHomePosition));
      }
      case GROUNDINTAKE -> {
        m_AlgaePivot.setControl(pivotRequest.withPosition(pivotGroundIntakePosition));
      }
      case DEAGLAEFY -> {
        m_AlgaePivot.setControl(pivotRequest.withPosition(pivotDeAlgifyPosition));
      }
      case NONE -> {
        // just keep the pivot retracted if there is no state
        m_AlgaePivot.setControl(pivotRequest.withPosition(pivotHomePosition));
      }
        // no default case because all states are accounted for

    }
  }
}
