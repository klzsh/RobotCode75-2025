// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.EndEffector;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.HardwareConstants;

enum IntakeState {
  NONE,
  INTAKING,
  HASGAMEPIECE,
  OUTAKING
}

/* IDEALLY (but doesn't have to be IE intaking -> outtaking, or none -> outtaking in case of error)
 * None -> Intaking -> (None, has game piece)
 * has game piece -> outaking -> none
 * 
 * Really, the only thing that is enforced is:
 * has game piece !-> intaking
 */

/*
 * The Intake serves a dual purpose. It intakes both Coral and Algae.
 *
 * The Coral taken in by a set of rollers driven by a (tentative) kraken x60.
 * There is a beam break sensor which will detect if there is a coral in the intake
 *
 * The algae intake also has two rollers driven by a (tentative) kraken X60.
 * The current applied to the stator will tell us when the algae is fully in the intake.
 *   There is no limit switch for the algae.
 */
public class Intake extends SubsystemBase {
  private static IntakeState m_CoralIntakeState;
  private static IntakeState m_AlgaeIntakeState;

  private TalonFX m_CoralMotor;
  private TalonFX m_AlgaeMotor;

  private DigitalInput m_CoralBeamBreak;

  /** Creates a new intake. */
  public Intake() {
    /* we might need to specify canbus */
    m_CoralMotor = new TalonFX(HardwareConstants.EndEffector.coralMotorCanID);
    m_AlgaeMotor = new TalonFX(HardwareConstants.EndEffector.algaeMotorCanID);

    m_CoralBeamBreak = new DigitalInput(HardwareConstants.EndEffector.coralBeamBreakCanID);

    m_CoralMotor.getConfigurator().apply(HardwareConstants.EndEffector.getCoralMotorConfiguration());
    m_AlgaeMotor.getConfigurator().apply(HardwareConstants.EndEffector.getAlgaeMotorConfiguration());
    
    m_CoralIntakeState = IntakeState.NONE;
    m_AlgaeIntakeState = IntakeState.NONE;
  }

  public void runCoralIntake(int RPM) {
    /* Cannot intake if HASGAMEPIECE */
    if (m_CoralIntakeState == IntakeState.HASGAMEPIECE) {
      return;
    }
    m_CoralIntakeState = IntakeState.INTAKING;

    
  }

  public void runCoralOutake(int RPM) {
    m_AlgaeIntakeState = IntakeState.OUTAKING;
  }

  public IntakeState getCoralState() {
    return m_CoralIntakeState;
  }

  public void runAlgaeIntake(int RPM) {
    /* Cannot intake if HASGAMEPIECE */
    if (m_AlgaeIntakeState == IntakeState.HASGAMEPIECE) {
      return;
    }
    m_AlgaeIntakeState = IntakeState.INTAKING;
  }

  public void runAlgaeOutake(int RPM) {
    m_AlgaeIntakeState = IntakeState.OUTAKING;
  }

  public IntakeState getAlgaeState() {
    return m_AlgaeIntakeState;
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
