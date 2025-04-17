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
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Importance;
import edu.wpi.first.epilogue.Logged.Strategy;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.dashboard.TunableNumber;
import frc.robot.subsystems.Util.LidarDistanceSensor;

@Logged(name = "Algae Intake", strategy = Strategy.OPT_IN, importance = Importance.CRITICAL)
public class AlgaeIntake extends SubsystemBase {
  public static enum AlgaeStates {
    NONE,
    INTAKING,
    HASGAMEPIECE,
    PROCESSOR,
    NET
  }

  @Logged(name = "Intake State", importance = Importance.CRITICAL)
  private AlgaeStates m_AlgaeIntakeState;

  // @Logged(name = "Algae Intake Motor", importance = Importance.INFO)
  private TalonFX m_AlgaeMotor;

  private Slot0Configs IntakePIDConfig = new Slot0Configs();
  private final TunableNumber algaeIntakeKp;
  private final TunableNumber algaeIntakeKd;
  private final TunableNumber algaeIntakeKs;

  public final TunableNumber intakeSpeed;
  public final TunableNumber processorSpeed;
  public final TunableNumber netSpeed;
  public final TunableNumber holdSpeed;
  @Logged private final LidarDistanceSensor m_AlgaeDetector;

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
    m_AlgaeDetector = new LidarDistanceSensor(Inches.of(3));

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

    intakeSpeed =
        new TunableNumber("Algae Intake/Intake Speed", algaeIntakeSpeed.in(RotationsPerSecond));
    processorSpeed =
        new TunableNumber(
            "Algae Intake/Processor Speed", algaeProcessorSpeed.in(RotationsPerSecond));
    netSpeed = new TunableNumber("Algae Intake/Net Speed", algaeNetSpeed.in(RotationsPerSecond));
    holdSpeed = new TunableNumber("Algae Intake/Hold Speed", algaeHoldSpeed.in(RotationsPerSecond));
  }

  public void setAlgaeState(AlgaeStates state) {
    if (m_AlgaeIntakeState == AlgaeStates.HASGAMEPIECE
        && state != AlgaeStates.PROCESSOR
        && state != AlgaeStates.NET) {
      m_AlgaeIntakeState = AlgaeStates.HASGAMEPIECE;
    } else {
      m_AlgaeIntakeState = state;
    }
  }

  @Logged(name = "Algae In Inake", importance = Importance.CRITICAL)
  public boolean algaeInIntake() {
    return m_AlgaeDetector.belowThreshold();
  }

  public AlgaeStates getAlgaeState() {
    return m_AlgaeIntakeState;
  }

  public void resetAlgaeState() {
    m_AlgaeIntakeState = AlgaeStates.NONE;
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("Has Algae", algaeInIntake());

    if (algaeIntakeKp.getNumber() != IntakePIDConfig.kP
        || algaeIntakeKd.getNumber() != IntakePIDConfig.kD
        || algaeIntakeKs.getNumber() != IntakePIDConfig.kS) {
      IntakePIDConfig.kP = algaeIntakeKp.getNumber();
      IntakePIDConfig.kD = algaeIntakeKd.getNumber();
      IntakePIDConfig.kS = algaeIntakeKs.getNumber();

      m_AlgaeMotor.getConfigurator().apply(IntakePIDConfig);
    }

    if (algaeInIntake()
        && m_AlgaeIntakeState != AlgaeStates.PROCESSOR
        && m_AlgaeIntakeState != AlgaeStates.NET) {
      m_AlgaeIntakeState = AlgaeStates.HASGAMEPIECE;
    }

    switch (m_AlgaeIntakeState) {
      case INTAKING -> {
        m_AlgaeMotor.setControl(algaeRequest.withVelocity(intakeSpeed.getNumber()));
      }
      case HASGAMEPIECE -> {
        // set the velocity control to a very low value (like 1-2 rps) to hold the algae in
        // m_AlgaeMotor.setControl(currentOut.withOutput(algaeHoldCurrent));
        m_AlgaeMotor.setControl(algaeRequest.withVelocity(holdSpeed.getNumber()));
      }
      case PROCESSOR -> {
        m_AlgaeMotor.setControl(algaeRequest.withVelocity(processorSpeed.getNumber()));
      }
      case NET -> {
        m_AlgaeMotor.setControl(algaeRequest.withVelocity(netSpeed.getNumber()));
      }
      case NONE -> {
        // set a NONE state for when there is no algae and we are not intaking anything
        m_AlgaeMotor.setControl(currentOut.withOutput(Amps.of(0)));
      }
        // no default case because all states are accounted for
    }
  }
}
