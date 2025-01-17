// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.EndEffector;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static frc.robot.Constants.EndEffectorConstants.*;
import static frc.robot.Constants.HardwareConstants.EndEffector.*;

import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CoralIntake extends SubsystemBase {
  public static enum CoralStates {
    // no intaking state because coral sits on top of the intake wheels
    HASGAMEPIECE,
    SCORING,
    DEFAULT // default is when the intake is doing nothing
  }

  private static CoralStates m_CoralIntakeState;
  private TalonFX m_CoralMotor;
  private DigitalInput m_CoralBeamBreak;
  // TODO: add tunable numbers for PID configs, velocity speed

  private final VelocityTorqueCurrentFOC velocityRequest =
      new VelocityTorqueCurrentFOC(RotationsPerSecond.of(0));
  private final TorqueCurrentFOC currentOut = new TorqueCurrentFOC(Amps.of(0));

  /** Creates a new CoralIntake. */
  public CoralIntake() {
    m_CoralMotor = new TalonFX(coralMotorCanID);
    m_CoralIntakeState = CoralStates.DEFAULT;
    m_CoralBeamBreak = new DigitalInput(coralBeamBreakDIO);

    velocityRequest.UpdateFreqHz = 0;
    velocityRequest.UseTimesync = true;

    currentOut.UpdateFreqHz = 0;
    currentOut.UseTimesync = true;

    m_CoralMotor.getConfigurator().apply(getCoralMotorConfiguration());
  }

  public void setState(CoralStates state) {
    m_CoralIntakeState = state;
  }

  public CoralStates getState() {
    return m_CoralIntakeState;
  }

  public CoralStates getCoralState() {
    return m_CoralIntakeState;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // TODO: check that the beam break returns true when it is broken
    if (m_CoralBeamBreak.get()) {
      m_CoralIntakeState = CoralStates.HASGAMEPIECE;
    }

    switch (m_CoralIntakeState) {
      case HASGAMEPIECE -> {
        // motors do not move, beam break is broken
        // state does not change the motor output, just states that there is a gamepeice
        // in the robot
        // this state should never be set by an external command
        m_CoralMotor.setControl(currentOut.withOutput(0));
      }
      case SCORING -> {
        m_CoralMotor.setControl(velocityRequest.withVelocity(coralScoreSpeed));
      }
      case DEFAULT -> {
        m_CoralMotor.setControl(currentOut.withOutput(0));
      }
    }
  }
}
