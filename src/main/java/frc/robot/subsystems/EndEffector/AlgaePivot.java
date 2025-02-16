// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.EndEffector;

import static edu.wpi.first.units.Units.*;
import static frc.robot.Constants.EndEffectorConstants.*;
import static frc.robot.Constants.EndEffectorConstants.MotorConfigs.*;
import static frc.robot.Constants.RobotConstants.*;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.MotionMagicExpoTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Importance;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.dashboard.TunableNumber;

public class AlgaePivot extends SubsystemBase {

  public static enum PivotState {
    RETRACTED,
    GROUNDINTAKE,
    DEALGAEFY,
    NONE
  }

  private PivotState m_PivotState;

  @Logged(name = "Algae Pivot Motor", importance = Importance.INFO)
  private TalonFX m_AlgaePivot;

  private Slot0Configs PivotPIDConfig = new Slot0Configs();
  private final TunableNumber algaePivotKp;
  private final TunableNumber algaePivotKd;
  private final TunableNumber algaePivotKs;
  private final TunableNumber algaePivotKg;

  private MotionMagicConfigs PivotMMConfigs = new MotionMagicConfigs();
  private final TunableNumber algaePivotMMAcc;
  private final TunableNumber algaePivotMMVel;
  private final TunableNumber algaePivotMMJerk;
  private final TunableNumber algaePivotMMKa;
  private final TunableNumber algaePivotMMKv;

  public final TunableNumber absoluteEncoderOffset;

  private final MotionMagicExpoTorqueCurrentFOC pivotRequest =
      new MotionMagicExpoTorqueCurrentFOC(Rotations.of(0));

  private final DutyCycleEncoder m_absoluteEncoder;

  public AlgaePivot() {
    m_AlgaePivot = new TalonFX(pivotCanID, superstructureCANBusName);
    m_AlgaePivot.getConfigurator().apply(getPivotConfiguration());
    m_PivotState = PivotState.NONE;

    pivotRequest.UpdateFreqHz = 0;
    pivotRequest.UseTimesync = true;

    PivotMMConfigs.withMotionMagicAcceleration(pivotMMAcc)
        .withMotionMagicCruiseVelocity(pivotMMVel)
        .withMotionMagicJerk(pivotMMJerk)
        .withMotionMagicExpo_kA(pivotMMKa)
        .withMotionMagicExpo_kV(pivotMMKv);

    algaePivotMMAcc = new TunableNumber("Algae Pivot/MMAcc", pivotMMAcc);
    algaePivotMMVel = new TunableNumber("Algae Pivot/MMVel", pivotMMVel);
    algaePivotMMJerk = new TunableNumber("Algae Pivot/MMJerk", pivotMMJerk);
    algaePivotMMKa = new TunableNumber("Algae Pivot/MMKa", pivotMMKa);
    algaePivotMMKv = new TunableNumber("Algae Pivot/MMKv", pivotMMKv);

    absoluteEncoderOffset =
        new TunableNumber("Algae Encoder/Offset", algaeEncoderOffset.in(Rotations));

    PivotPIDConfig.withKS(pivotKS)
        .withKG(pivotKG)
        .withKP(pivotKP)
        .withKD(pivotKD)
        .withGravityType(GravityTypeValue.Arm_Cosine);

    algaePivotKp = new TunableNumber("Algae Pivot/kP", pivotKP);
    algaePivotKd = new TunableNumber("Algae Pivot/kD", pivotKD);
    algaePivotKg = new TunableNumber("Algae Pivot/kG", pivotKG);
    algaePivotKs = new TunableNumber("Algae Pivot/kS", pivotKS);

    m_absoluteEncoder =
        new DutyCycleEncoder(algaePivotEncoderPort, 1, algaePivotZeroPoint.in(Rotations));
    Timer.delay(5);
    m_AlgaePivot.setPosition((getAbsolutePosition() + 0.01) * pivotMotorGearRatio);
  }

  public void setPivotState(PivotState state) {
    m_PivotState = state;
  }

  public PivotState getPivotState() {
    return m_PivotState;
  }

  public void homePivotToAbsoluteEncoder() {
    double absoluteRotations = m_absoluteEncoder.get();
    double offset = absoluteEncoderOffset.getNumber();
    double relativeRotationsAxleCandidate1 = offset - absoluteRotations;
    double relativeRotationsAxleCandidate2 = offset - (absoluteRotations + 1);
    double relativeRotationsAxleCandidate3 = offset - (absoluteRotations - 1);

    double relativeRotationsAxle =
        Math.min(
            Math.abs(relativeRotationsAxleCandidate1),
            Math.min(
                Math.abs(relativeRotationsAxleCandidate2),
                Math.abs(relativeRotationsAxleCandidate3)));

    double relativeRotationsMotor = relativeRotationsAxle * pivotMotorGearRatio;

    // find rotations from current relative position to home
    double totalRotations =
        m_AlgaePivot.getPosition().getValue().in(Rotations) - relativeRotationsMotor;

    m_AlgaePivot.setControl(pivotRequest.withPosition(Rotations.of(totalRotations)));
  }

  public void resetPivotState() {
    m_PivotState = PivotState.NONE;
  }

  public boolean isAtPositionAbsolute(double absolutePosition) {
    return Math.abs(absolutePosition - m_absoluteEncoder.get()) < algaePivotDeadband;
  }

  public void resetPivotMotor(Angle rotations) {
    m_AlgaePivot.setPosition(rotations);
  }

  @Logged
  public double getAbsolutePosition() {
    return m_absoluteEncoder.get();
  }

  @Logged(name = "Pivot Position")
  public double getPivotPosition() {
    return m_AlgaePivot.getPosition().refresh().getValue().in(Rotations);
  }

  @Override
  public void periodic() {
    if (algaePivotKp.getNumber() != PivotPIDConfig.kP
        || algaePivotKd.getNumber() != PivotPIDConfig.kD
        || algaePivotKs.getNumber() != PivotPIDConfig.kS
        || algaePivotKg.getNumber() != PivotPIDConfig.kG) {
      PivotPIDConfig.kP = algaePivotKp.getNumber();
      PivotPIDConfig.kD = algaePivotKd.getNumber();
      PivotPIDConfig.kS = algaePivotKs.getNumber();
      PivotPIDConfig.kG = algaePivotKg.getNumber();

      m_AlgaePivot.getConfigurator().apply(PivotPIDConfig);
    }

    if (algaePivotMMAcc.getNumber() != PivotMMConfigs.MotionMagicAcceleration
        || algaePivotMMVel.getNumber() != PivotMMConfigs.MotionMagicCruiseVelocity
        || algaePivotMMJerk.getNumber() != PivotMMConfigs.MotionMagicJerk
        || algaePivotMMKv.getNumber() != PivotMMConfigs.MotionMagicExpo_kV
        || algaePivotMMKa.getNumber() != PivotMMConfigs.MotionMagicExpo_kA) {
      PivotMMConfigs.MotionMagicAcceleration = algaePivotMMAcc.getNumber();
      PivotMMConfigs.MotionMagicCruiseVelocity = algaePivotMMVel.getNumber();
      PivotMMConfigs.MotionMagicJerk = algaePivotMMJerk.getNumber();
      PivotMMConfigs.MotionMagicExpo_kV = algaePivotMMKv.getNumber();
      PivotMMConfigs.MotionMagicExpo_kA = algaePivotMMKv.getNumber();

      m_AlgaePivot.getConfigurator().apply(PivotMMConfigs);
    }

    switch (m_PivotState) {
      case RETRACTED -> {
        m_AlgaePivot.setControl(pivotRequest.withPosition(pivotHomePosition));
      }
      case GROUNDINTAKE -> {
        m_AlgaePivot.setControl(pivotRequest.withPosition(pivotGroundIntakePosition));
      }
      case DEALGAEFY -> {
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
