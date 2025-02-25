// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.EndEffector;

import static edu.wpi.first.units.Units.*;
import static frc.robot.Constants.ElevatorConstants.*;
import static frc.robot.Constants.ElevatorConstants.MotorConfigs.*;
import static frc.robot.Constants.RobotConstants.superstructureCANBusName;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.controls.DynamicMotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/*
 * Cascading elevator driven by 2 Kraken X60s
 * 2 stage WCP GreyT elevator
 * Uses motion magic to honor a target cruise velocity and acceleration
 *  Motion magic is used to prevent the elevator from destroying itself by moving too fast
 *
 */
// @Logged(strategy = Strategy.OPT_IN, name = "Elevator")
public class Elevator extends SubsystemBase {

  // we only have a certain number of states the elevator will be in at any given time
  public static enum ElevatorPositions {
    L1(l1Position),
    L2(l2Position),
    L3(l3Position),
    L4(l4Position),
    HOME(homePosition),
    PROCESSOR(processorPosition);

    public final Angle Rotations;

    private ElevatorPositions(Angle rotations) {
      this.Rotations = rotations;
    }
  }

  // @Logged(name = "Current Position", importance = Importance.INFO)
  private ElevatorPositions m_SetpointPosition = ElevatorPositions.HOME;

  // @Logged
  private boolean m_IsAlgae = false;

  // define motors
  // @Logged(name = "Left Elevator Motor", importance = Importance.DEBUG)
  private final TalonFX m_ElevatorMotor1;

  // @Logged(name = "Right Elevator Motor", importance = Importance.DEBUG)
  private final TalonFX m_ElevatorMotor2;

  // sensors
  private final DigitalInput m_lowerLimitSwitch;
  private final DigitalInput m_upperLimitSwitch;
  private final DigitalInput m_backupLimitSwitch;

  // define control requests
  private final DynamicMotionMagicTorqueCurrentFOC m_PositionRequest;
  private final TorqueCurrentFOC m_CharacterizationRequest;

  public Elevator() {
    // initialize motors, using the non drivetrain CANivore bus
    m_ElevatorMotor1 = new TalonFX(elevatorMotor1CANID, superstructureCANBusName);
    m_ElevatorMotor2 = new TalonFX(elevatorMotor2CANID, superstructureCANBusName);
    // initialize sensors
    m_lowerLimitSwitch = new DigitalInput(lowerLimitPort);
    m_upperLimitSwitch = new DigitalInput(upperLimitPort);
    m_backupLimitSwitch = new DigitalInput(backupLimitPort);
    // m_distanceSensor = new Counter(distanceSensorPort);
    // initialize control requests
    m_PositionRequest = new DynamicMotionMagicTorqueCurrentFOC(0, 0, 0, 0);
    m_CharacterizationRequest = new TorqueCurrentFOC(Amps.of(0));
    // configure motors with correct inverts
    m_ElevatorMotor1.getConfigurator().apply(getElevatorMotorConfig());
    m_ElevatorMotor2.getConfigurator().apply(getElevatorMotorConfig());
    // set the position of the elevator
    m_ElevatorMotor1.setPosition(ElevatorPositions.HOME.Rotations);
    m_ElevatorMotor2.setPosition(ElevatorPositions.HOME.Rotations);

    m_CharacterizationRequest.UpdateFreqHz = 0;
    m_CharacterizationRequest.UseTimesync = true;

    m_PositionRequest.UpdateFreqHz = 0;
    m_PositionRequest.UseTimesync = true;
  }

  /**
   * sets the elevator state
   *
   * @param position the position to set the elevator to
   */
  public void setPosition(ElevatorPositions position, boolean isAlgae) {
    m_SetpointPosition = position;
    m_IsAlgae = isAlgae;
  }

  /**
   * average position between the two motors
   *
   * @return the position in rotations of the elevator
   */
  // @Logged(name = "Elevator Position Radians")
  public Angle getPosition() {
    Measure<AngleUnit> motor1Position =
        BaseStatusSignal.getLatencyCompensatedValue(
            m_ElevatorMotor1.getPosition(), m_ElevatorMotor1.getVelocity());
    Measure<AngleUnit> motor2Position =
        BaseStatusSignal.getLatencyCompensatedValue(
            m_ElevatorMotor2.getPosition(), m_ElevatorMotor2.getVelocity());

    return Rotations.of((motor1Position.in(Rotations) + motor2Position.in(Rotations)) / 2);
  }

  /**
   * logs elevator position in rotations
   *
   * @return the position in rotations of the elevator
   */
  // @Logged(name = "Elevator Position")
  public double logPosition() {
    return getPosition().in(Rotations);
  }

  /**
   * checks if the elevator is at the correct position, including if the elevator is going to the
   * algae position
   *
   * @param position position to check
   * @param isAlgae apply the algae offset
   * @return if the elevator is at the correct position
   */
  public boolean isAtPosition(ElevatorPositions position, boolean isAlgae) {
    if (position == ElevatorPositions.HOME) {
      return getLowerLimit();
    }
    double currentPosition = getPosition().in(Rotations);
    double algaeOffset =
        (isAlgae && (position == ElevatorPositions.L2 || position == ElevatorPositions.L3))
            ? algaeRemovalOffset.in(Rotations)
            : 0;
    return MathUtil.applyDeadband(
            currentPosition - position.Rotations.in(Rotations) - algaeOffset,
            deadband.in(Rotations))
        == 0.0;
  }

  public boolean getUpperLimit() {
    return !m_upperLimitSwitch.get();
  }

  public boolean getLowerLimit() {
    return !m_lowerLimitSwitch.get() || !m_backupLimitSwitch.get();
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("Upper Limit", getUpperLimit());
    SmartDashboard.putBoolean("Lower Limit", getLowerLimit());

    // if (getLowerLimit()) {
    //   m_ElevatorMotor1.setPosition(Rotations.of(0));
    //   m_ElevatorMotor2.setPosition(Rotations.of(0));
    // }
    // if (getUpperLimit()) {
    //   m_ElevatorMotor1.setPosition(Rotations.of(26));
    //   m_ElevatorMotor2.setPosition(Rotations.of(26));
    // }
    double currentPosition = m_SetpointPosition.Rotations.in(Rotations);
    double algaeOffset =
        (m_IsAlgae
                && (m_SetpointPosition == ElevatorPositions.L2
                    || m_SetpointPosition == ElevatorPositions.L3))
            ? algaeRemovalOffset.in(Rotations)
            : 0;
    double targetRotations = currentPosition + algaeOffset;

    if (getPosition().in(Rotations) < targetRotations) {
      m_PositionRequest.Velocity = MotionMagicProfileUp[0];
      m_PositionRequest.Acceleration = MotionMagicProfileUp[1];
      m_PositionRequest.Jerk = MotionMagicProfileUp[2];
    } else {
      m_PositionRequest.Velocity = MotionMagicProfileDown[0];
      m_PositionRequest.Acceleration = MotionMagicProfileDown[1];
      m_PositionRequest.Jerk = MotionMagicProfileDown[2];
    }

    if (getLowerLimit() && m_SetpointPosition == ElevatorPositions.HOME) {
      m_ElevatorMotor1.setControl(m_CharacterizationRequest.withOutput(0));
      m_ElevatorMotor2.setControl(m_CharacterizationRequest.withOutput(0));
    } else {
      m_ElevatorMotor1.setControl(
          m_PositionRequest
              .withPosition(Rotations.of(targetRotations))
              .withLimitForwardMotion(getUpperLimit())
              .withLimitReverseMotion(getLowerLimit()));
      m_ElevatorMotor2.setControl(
          m_PositionRequest
              .withPosition(Rotations.of(targetRotations))
              .withLimitForwardMotion(getUpperLimit())
              .withLimitReverseMotion(getLowerLimit()));
    }
  }
}
