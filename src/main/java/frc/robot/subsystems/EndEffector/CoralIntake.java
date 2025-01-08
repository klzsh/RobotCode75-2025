// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.EndEffector;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.math.Conversions;
import frc.robot.Constants.HardwareConstants;

public class CoralIntake extends SubsystemBase {
  private static IntakeState m_CoralIntakeState;
  private TalonFX m_CoralMotor;
  private DigitalInput m_CoralBeamBreak;

  private final VelocityTorqueCurrentFOC torqueVelocity = new VelocityTorqueCurrentFOC(RotationsPerSecond.of(0));

  /** Creates a new CoralIntake. */
  public CoralIntake() {
    m_CoralMotor = new TalonFX(HardwareConstants.EndEffector.coralMotorCanID);
    m_CoralIntakeState = IntakeState.NONE;
    m_CoralBeamBreak = new DigitalInput(HardwareConstants.EndEffector.coralBeamBreakCanID);

    m_CoralMotor.getConfigurator().apply(HardwareConstants.EndEffector.getCoralMotorConfiguration());
  }

  public void runCoral(double RPM) {
    /* Eject if HASGAMEPIECE */
    m_CoralIntakeState = (m_CoralIntakeState == IntakeState.HASGAMEPIECE) ? IntakeState.OUTAKING : IntakeState.INTAKING;
    m_CoralMotor.setControl(torqueVelocity.withVelocity(RotationsPerSecond.of(RPM * 60)));
  }

  public IntakeState getCoralState() {
    return m_CoralIntakeState;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (Math.abs(m_CoralMotor.getVelocity().getValue().in(RotationsPerSecond)) < 0.1) {
      m_CoralIntakeState = m_CoralBeambreak.get() ? IntakeState.HASGAMEPIECE : IntakeState.NONE;
    }
  }
}
