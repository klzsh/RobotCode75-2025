// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.EndEffector;

import static edu.wpi.first.units.Units.*;
import static frc.robot.Constants.EndEffectorConstants.*;
import static frc.robot.Constants.EndEffectorConstants.MotorConfigs.*;
import static frc.robot.Constants.RobotConstants.*;

import com.ctre.phoenix6.controls.MotionMagicExpoTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Importance;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.dashboard.TunableNumber;

public class AlgaePivot extends SubsystemBase {

  public static enum PivotState {
    RETRACTED(pivotHomePosition),
    GROUNDINTAKE(pivotGroundIntakePosition),
    DEALGAEFY(pivotDeAlgifyPosition),
    NONE(pivotHomePosition);

    public final Angle Rotations;

    private PivotState(Angle rotations) {
      this.Rotations = rotations;
    }
  }

  private PivotState m_PivotState;

  @Logged(name = "Algae Pivot Motor", importance = Importance.INFO)
  private TalonFX m_AlgaePivot;

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

    absoluteEncoderOffset =
        new TunableNumber("Algae Encoder/Offset", algaeEncoderOffset.in(Rotations));

    m_absoluteEncoder =
        new DutyCycleEncoder(algaePivotEncoderPort, 1, algaePivotZeroPoint.in(Rotations));
    Timer.delay(5);
    m_AlgaePivot.setPosition((getAbsolutePosition() - 0.135) * pivotMotorGearRatio);
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

  public boolean isAtPosition(PivotState state) {
    return Math.abs(
            state.Rotations.in(Rotations) - m_AlgaePivot.getPosition().getValue().in(Rotations))
        < algaePivotDeadband;
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
    m_AlgaePivot.setControl(pivotRequest.withPosition(m_PivotState.Rotations));
  }
}
