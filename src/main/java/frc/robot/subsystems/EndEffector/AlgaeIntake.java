// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.EndEffector;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.HardwareConstants;

enum IntakeState {
  NONE,
  INTAKING,
  HASGAMEPIECE,
  OUTAKING
}

public class AlgaeIntake extends SubsystemBase {

  private static IntakeState m_AlgaeIntakeState;
  private TalonFX m_AlgaeMotor;

  private final VelocityTorqueCurrentFOC torqueVelocity =
      new VelocityTorqueCurrentFOC(RotationsPerSecond.of(0));
  private final TorqueCurrentFOC currentOut = new TorqueCurrentFOC(Amps.of(0));

  /** Creates a new AlgaeIntake. */
  public AlgaeIntake() {
    m_AlgaeMotor = new TalonFX(HardwareConstants.EndEffector.algaeMotorCanID);
    m_AlgaeMotor
        .getConfigurator()
        .apply(HardwareConstants.EndEffector.getAlgaeMotorConfiguration());
    m_AlgaeIntakeState = IntakeState.NONE;

    torqueVelocity.UpdateFreqHz = 0;
    torqueVelocity.UseTimesync = true;

    currentOut.UpdateFreqHz = 0;
    currentOut.UseTimesync = true;
  }

  public void runAlgaeIntake(double RPM) {
    /* Cannot intake if HASGAMEPIECE */
    m_AlgaeIntakeState = IntakeState.INTAKING;
    m_AlgaeMotor.setControl(torqueVelocity.withVelocity(RotationsPerSecond.of(RPM * 60)));
  }

  public void runAlgaeOutake(double RPM) {
    m_AlgaeIntakeState = IntakeState.OUTAKING;
    m_AlgaeMotor.setControl(torqueVelocity.withVelocity(RotationsPerSecond.of(-RPM * 60)));
  }

  public IntakeState getAlgaeState() {
    return m_AlgaeIntakeState;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (m_AlgaeMotor.getFault_StatorCurrLimit().getValue()) {
      m_AlgaeIntakeState = IntakeState.HASGAMEPIECE;
    }
  }
}
