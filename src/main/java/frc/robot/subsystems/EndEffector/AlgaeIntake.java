// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.EndEffector;

import static edu.wpi.first.units.Units.*;
import static frc.robot.Constants.EndEffectorConstants.*;
import static frc.robot.Constants.EndEffectorConstants.MotorConfigs.*;
import static frc.robot.Constants.RobotConstants.superstructureCANBusName;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.dashboard.TunableNumber;
import frc.robot.subsystems.Util.LidarDistance;

// @Logged(name = "Algae Intake", strategy = Strategy.OPT_IN)
public class AlgaeIntake extends SubsystemBase {
  public static enum AlgaeStates {
    NONE,
    INTAKING,
    HASGAMEPIECE,
    OUTAKING
  }

  // TODO: add tunable numbers for PIDS, velocity, Position, CURRENT LIMITS
  // @Logged(name = "Intake State")
  private AlgaeStates m_AlgaeIntakeState;

  // @Logged(name = "Algae Intake Motor", importance = Importance.INFO)
  private TalonFX m_AlgaeMotor;

  private Slot0Configs IntakePIDConfig = new Slot0Configs();
  private final TunableNumber algaeIntakeKp;
  private final TunableNumber algaeIntakeKd;
  private final TunableNumber algaeIntakeKs;

  // private final DigitalInput m_AlgaeIntakeLimit;
  // @Logged
  private final LidarDistance m_AlgaeDetector;

  // intake speed
  private final VelocityTorqueCurrentFOC algaeRequest =
      new VelocityTorqueCurrentFOC(RotationsPerSecond.of(0));
  // hold request
  private final TorqueCurrentFOC currentOut = new TorqueCurrentFOC(Amps.of(0));

  /** Creates a new AlgaeIntake. */
  public AlgaeIntake() {
    m_AlgaeMotor = new TalonFX(algaeMotorCanID, superstructureCANBusName);
    m_AlgaeMotor.getConfigurator().apply(getAlgaeMotorConfiguration());
    m_AlgaeIntakeState = AlgaeStates.NONE;
    m_AlgaeDetector = new LidarDistance(Inches.of(4));

    // m_AlgaeIntakeLimit = new DigitalInput(algaeLimitSwitchPort);

    algaeRequest.UpdateFreqHz = 0;
    algaeRequest.UseTimesync = true;

    currentOut.UpdateFreqHz = 0;
    currentOut.UseTimesync = true;

    IntakePIDConfig.withKA(0)
        .withKS(algaeKS)
        .withKP(algaeKP)
        .withKD(algaeKD)
        .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseVelocitySign);

    algaeIntakeKp = new TunableNumber("Algae Intake/kP", algaeKP);
    algaeIntakeKd = new TunableNumber("Algae Intake/kD", algaeKD);
    algaeIntakeKs = new TunableNumber("Algae Intake/kS", algaeKS);
  }

  public void setAlgaeState(AlgaeStates state) {
    if (m_AlgaeIntakeState == AlgaeStates.HASGAMEPIECE && state != AlgaeStates.OUTAKING) {
      m_AlgaeIntakeState = AlgaeStates.HASGAMEPIECE;
    } else {
      m_AlgaeIntakeState = state;
    }
  }

  // @Logged
  public boolean algaeInIntake() {
    return m_AlgaeDetector.belowThreshold();
    // return !m_AlgaeIntakeLimit.get();
  }

  public AlgaeStates getAlgaeState() {
    return m_AlgaeIntakeState;
  }

  public void resetAlgaeState() {
    m_AlgaeIntakeState = AlgaeStates.NONE;
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

    if (algaeInIntake() && m_AlgaeIntakeState != AlgaeStates.OUTAKING) {
      m_AlgaeIntakeState = AlgaeStates.HASGAMEPIECE;
    }

    switch (m_AlgaeIntakeState) {
      case INTAKING -> {
        m_AlgaeMotor.setControl(algaeRequest.withVelocity(algaeIntakeSpeed));
      }
      case HASGAMEPIECE -> {
        // set the velocity control to a very low value (like 1-2 rps) to hold the algae in
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
  }
}
