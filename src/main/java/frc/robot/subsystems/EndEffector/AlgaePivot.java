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
import edu.wpi.first.epilogue.Logged.Strategy;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

@Logged(name = "Algae Pivot", strategy = Strategy.OPT_IN, importance = Importance.CRITICAL)
public class AlgaePivot extends SubsystemBase {

  public static enum PivotState {
    RETRACTED(pivotHomePosition),
    GROUNDINTAKE(pivotGroundIntakePosition),
    DEALGAEFY(pivotDeAlgifyPosition),
    PROCESSOR(pivotProcessorPosition),
    NET(pivotNetPosition),
    NONE(pivotHomePosition);

    public final Angle Rotations;

    private PivotState(Angle rotations) {
      this.Rotations = rotations;
    }
  }

  private PivotState m_PivotState;

  // @Logged(name = "Algae Pivot Motor", importance = Importance.INFO)
  private TalonFX m_AlgaePivot;

  // public final TunableNumber homePosition;
  // public final TunableNumber groundIntakePosition;
  // public final TunableNumber deAlgifyPosition;
  // public final TunableNumber processorPosition;
  // public final TunableNumber netPosition;

  private final MotionMagicExpoTorqueCurrentFOC pivotRequest =
      new MotionMagicExpoTorqueCurrentFOC(Rotations.of(0));

  private final DutyCycleEncoder m_absoluteEncoder;

  public AlgaePivot() {
    m_AlgaePivot = new TalonFX(pivotCanID, superstructureCANBusName);
    m_AlgaePivot.getConfigurator().apply(getPivotConfiguration());
    m_PivotState = PivotState.NONE;

    pivotRequest.UpdateFreqHz = 0;
    pivotRequest.UseTimesync = true;

    m_absoluteEncoder =
        new DutyCycleEncoder(algaePivotEncoderPort, 1, algaePivotZeroPoint.in(Rotations));
    Timer.delay(5);
    m_AlgaePivot.setPosition(
        (getAbsolutePosition() - pivotEncoderOffset.in(Rotations)) * pivotMotorGearRatio);

    // homePosition = new TunableNumber("Algae Pivot/Home Position",
    // pivotHomePosition.in(Rotations));
    // groundIntakePosition =
    //     new TunableNumber(
    //         "Algae Pivot/Ground Intake Position", pivotGroundIntakePosition.in(Rotations));
    // deAlgifyPosition =
    //     new TunableNumber("Algae Pivot/DeAlgify Position", pivotDeAlgifyPosition.in(Rotations));
    // processorPosition =
    //     new TunableNumber("Algae Pivot/Processor Position",
    // pivotProcessorPosition.in(Rotations));
    // netPosition = new TunableNumber("Algae Pivot/Net Position", pivotNetPosition.in(Rotations));
  }

  public void setPivotState(PivotState state) {
    m_PivotState = state;
  }

  @Logged(name = "Pivot State", importance = Importance.CRITICAL)
  public PivotState getPivotState() {
    return m_PivotState;
  }

  public void resetPivotState() {
    m_PivotState = PivotState.NONE;
  }

  public double getPivotDelay() {
    // return PivotRetractDelay.getNumber();
    return 0.4;
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

  @Logged(name = "Pivot Position", importance = Importance.CRITICAL)
  public double getPivotPosition() {
    return m_AlgaePivot.getPosition().refresh().getValue().in(Rotations);
  }

  @Override
  public void periodic() {
    // if(m_PivotState == PivotState.DEALGAEFY){
    // m_AlgaePivot.setControl(pivotRequest.withPosition(deAlgaefyRotations.getNumber()))
    // } else {
    // switch (m_PivotState) {
    //   case RETRACTED ->
    //       m_AlgaePivot.setControl(pivotRequest.withPosition(homePosition.getNumber()));
    //   case GROUNDINTAKE ->
    //       m_AlgaePivot.setControl(pivotRequest.withPosition(groundIntakePosition.getNumber()));
    //   case DEALGAEFY ->
    //       m_AlgaePivot.setControl(pivotRequest.withPosition(deAlgifyPosition.getNumber()));
    //   case PROCESSOR ->
    //       m_AlgaePivot.setControl(pivotRequest.withPosition(processorPosition.getNumber()));
    //   case NET -> m_AlgaePivot.setControl(pivotRequest.withPosition(netPosition.getNumber()));
    //   case NONE -> m_AlgaePivot.setControl(pivotRequest.withPosition(homePosition.getNumber()));
    // }
    m_AlgaePivot.setControl(pivotRequest.withPosition(m_PivotState.Rotations));
    // }
  }
}
