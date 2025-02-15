// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.EndEffector;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static frc.robot.Constants.EndEffectorConstants.*;
import static frc.robot.Constants.HardwareConstants.EndEffector.*;
import static frc.robot.Constants.HardwareConstants.superstructureCANBusName;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.MotionMagicExpoTorqueCurrentFOC;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Importance;
import edu.wpi.first.epilogue.Logged.Strategy;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.dashboard.TunableNumber;

@Logged(name = "Algae Intake", strategy = Strategy.OPT_IN)
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
    DEALGAEFY,
    NONE
  }

  // 50 over 26 + 25:1 5.0*5.0*50.0/26.0
  // TODO: add tunable numbers for PIDS, velocity, Position, CURRENT LIMITS
  @Logged(name = "Intake State")
  private AlgaeStates m_AlgaeIntakeState;

  private PivotState m_PivotState;

  @Logged(name = "Algae Intake Motor", importance = Importance.INFO)
  private TalonFX m_AlgaeMotor;

  @Logged(name = "Algae Pivot Motor", importance = Importance.INFO)
  private TalonFX m_AlgaePivot;

  private Slot0Configs IntakePIDConfig = new Slot0Configs();
  private final TunableNumber algaeIntakeKp;
  private final TunableNumber algaeIntakeKd;
  private final TunableNumber algaeIntakeKs;

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

  private final VelocityTorqueCurrentFOC algaeRequest =
      new VelocityTorqueCurrentFOC(RotationsPerSecond.of(0));
  private final TorqueCurrentFOC currentOut = new TorqueCurrentFOC(Amps.of(0));
  private final MotionMagicExpoTorqueCurrentFOC pivotRequest =
      new MotionMagicExpoTorqueCurrentFOC(Rotations.of(0));

  private final DutyCycleEncoder m_absoluteEncoder;

  /** Creates a new AlgaeIntake. */
  public AlgaeIntake() {
    m_AlgaeMotor = new TalonFX(algaeMotorCanID, superstructureCANBusName);
    m_AlgaeMotor.getConfigurator().apply(getAlgaeMotorConfiguration());
    m_AlgaeIntakeState = AlgaeStates.NONE;

    m_AlgaePivot = new TalonFX(pivotCanID, superstructureCANBusName);
    m_AlgaePivot.getConfigurator().apply(getPivotConfiguration());
    m_PivotState = PivotState.NONE;

    algaeRequest.UpdateFreqHz = 0;
    algaeRequest.UseTimesync = true;

    currentOut.UpdateFreqHz = 0;
    currentOut.UseTimesync = true;

    pivotRequest.UpdateFreqHz = 0;
    pivotRequest.UseTimesync = true;

    IntakePIDConfig.withKA(0)
        .withKS(algaeKS)
        .withKV(0)
        .withKG(0)
        .withKP(algaeKP)
        .withKD(algaeKD)
        .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseVelocitySign);

    algaeIntakeKp = new TunableNumber("Algae Intake/kP", algaeKP);
    algaeIntakeKd = new TunableNumber("Algae Intake/kD", algaeKD);
    algaeIntakeKs = new TunableNumber("Algae Intake/kS", algaeKS);

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
        new DutyCycleEncoder(algaePivotEncoderPort, 1, algaePivotZeroPoint.getRotations());
    Timer.delay(5);
    m_AlgaePivot.setPosition((getAbsolutePosition() + 0.01) * pivotMotorGearRatio);
  }

  public void setAlgaeState(AlgaeStates state) {
    if (m_AlgaeIntakeState == AlgaeStates.HASGAMEPIECE && state != AlgaeStates.OUTAKING) {
      m_AlgaeIntakeState = AlgaeStates.HASGAMEPIECE;
    } else {
      m_AlgaeIntakeState = state;
    }
  }

  public void setStates(AlgaeStates algaeState, PivotState pivotState) {
    setAlgaeState(algaeState);
    setPivotState(pivotState);
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

  public void resetStates() {
    resetAlgaeState();
    resetPivotState();
  }

  public void resetAlgaeState() {
    if (m_AlgaeIntakeState != AlgaeStates.HASGAMEPIECE) {
      m_AlgaeIntakeState = AlgaeStates.NONE;
    }
  }

  public void resetPivotState() {
    m_PivotState = PivotState.NONE;
  }

  public boolean isAtPositionAbsolute(double absolutePosition) {
    return Math.abs(absolutePosition - m_absoluteEncoder.get()) < algaePivotDeadband;
  }


  public void runSetpoint() {
    m_AlgaePivot.setControl(pivotRequest.withPosition(1));
  }

  public void setZero() {
    m_AlgaePivot.setControl(pivotRequest.withPosition(9));
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
    if (algaeIntakeKp.getNumber() != IntakePIDConfig.kP
        || algaeIntakeKd.getNumber() != IntakePIDConfig.kD
        || algaeIntakeKs.getNumber() != IntakePIDConfig.kS) {
      IntakePIDConfig.kP = algaeIntakeKp.getNumber();
      IntakePIDConfig.kD = algaeIntakeKd.getNumber();
      IntakePIDConfig.kS = algaeIntakeKs.getNumber();

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

    // This method will be called once per scheduler run
    if (m_AlgaeMotor.getStatorCurrent(true).getValue().in(Amps) >= 38 && m_AlgaeIntakeState != AlgaeStates.OUTAKING) {
      m_AlgaeIntakeState = AlgaeStates.HASGAMEPIECE;
    }

    switch (m_AlgaeIntakeState) {
      case INTAKING -> {
        m_AlgaeMotor.setControl(algaeRequest.withVelocity(algaeIntakeSpeed));
      }
      case HASGAMEPIECE -> {
        // set the velocity control to a very low value (like 1-2 rps) to hold the algae in. The
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

    // switch (m_PivotState) {
    //   case RETRACTED -> {
    //     m_AlgaePivot.setControl(pivotRequest.withPosition(pivotHomePosition));
    //   }
    //   case GROUNDINTAKE -> {
    //     m_AlgaePivot.setControl(pivotRequest.withPosition(pivotGroundIntakePosition));
    //   }
    //   case DEALGAEFY -> {
    //     m_AlgaePivot.setControl(pivotRequest.withPosition(pivotDeAlgifyPosition));
    //   }
    //   case NONE -> {
    //     // just keep the pivot retracted if there is no state
    //     m_AlgaePivot.setControl(pivotRequest.withPosition(pivotHomePosition));
    //   }
    //   // no default case because all states are accounted for
    // }
  }
}
