// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.EndEffector;

import static edu.wpi.first.units.Units.*;
import static frc.robot.Constants.ElevatorConstants.*;
import static frc.robot.Constants.HardwareConstants.Elevator.*;
import static frc.robot.Constants.HardwareConstants.superstructureCANBusName;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.DynamicMotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Importance;
import edu.wpi.first.epilogue.Logged.Strategy;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.dashboard.TunableNumber;

/*
 * Cascading elevator driven by 2 Kraken X60s
 * 2 stage WCP GreyT elevator
 * Uses motion magic to honor a target cruise velocity and acceleration
 *  Motion magic is used to prevent the elevator from destroying itself by moving too fast
 *
 */
@Logged(strategy = Strategy.OPT_IN, name = "Elevator")
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

  @Logged(name = "Current Position", importance = Importance.INFO)
  private ElevatorPositions m_SetpointPosition = ElevatorPositions.HOME;

  private boolean m_IsAlgae = false;

  // define motors
  @Logged(name = "Left Elevator Motor", importance = Importance.DEBUG)
  private final TalonFX m_ElevatorMotor1;

  @Logged(name = "Right Elevator Motor", importance = Importance.DEBUG)
  private final TalonFX m_ElevatorMotor2;

  // sensors
  private final DigitalInput m_lowerLimitSwitch;
  private final DigitalInput m_upperLimitSwitch;
  private final DigitalInput m_backupLimitSwitch;
  // this is probably not going to be used for homing the elevator
  // private final Counter m_distanceSensor;

  // define control requests
  private final DynamicMotionMagicTorqueCurrentFOC m_PositionRequest;
  private final TorqueCurrentFOC m_CharacterizationRequest;

  // tunable numbers
  private final TunableNumber upVelocity;
  private final TunableNumber upAcceleration;
  private final TunableNumber upJerk;

  private final TunableNumber downVelocity;
  private final TunableNumber downAcceleration;
  private final TunableNumber downJerk;

  private Slot0Configs PIDConfig = new Slot0Configs();
  private final TunableNumber elevatorKp;
  private final TunableNumber elevatorKd;
  private final TunableNumber elevatorKg;
  private final TunableNumber elevatorKs;
  private final TunableNumber elevatorKa;
  private final TunableNumber elevatorKv;

  private final TunableNumber setpoint;
  private final TunableNumber setpoint1;
  private final TunableNumber setpoint2;

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

    // sets the default values for the tunable numbers
    upVelocity = new TunableNumber("Elevator/UpVelocity", MotionMagicProfileUp[0]);
    upAcceleration = new TunableNumber("Elevator/UpAcceleration", MotionMagicProfileUp[1]);
    upJerk = new TunableNumber("Elevator/UpJerk", MotionMagicProfileUp[2]);

    downVelocity = new TunableNumber("Elevator/DownVelocity", MotionMagicProfileDown[0]);
    downAcceleration = new TunableNumber("Elevator/DownAcceleration", MotionMagicProfileDown[1]);
    downJerk = new TunableNumber("Elevator/DownJerk", MotionMagicProfileDown[2]);

    setpoint = new TunableNumber("Elevator/Setpoint", 0);
    setpoint1 = new TunableNumber("Elevator/Setpoint1", 10);
    setpoint2 = new TunableNumber("Elevator/Setpoint2", 5);

    PIDConfig.withKA(kA)
        .withKS(kS)
        .withKV(kV)
        .withKG(kG)
        .withKP(kP)
        .withKD(kD)
        .withGravityType(GravityTypeValue.Elevator_Static)
        .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseVelocitySign);

    elevatorKp = new TunableNumber("Elevator/kP", kP);
    elevatorKd = new TunableNumber("Elevator/kD", kD);
    elevatorKg = new TunableNumber("Elevator/kG", kG);
    elevatorKs = new TunableNumber("Elevator/kS", kS);
    elevatorKa = new TunableNumber("Elevator/kA", kA);
    elevatorKv = new TunableNumber("Elevator/kV", kV);

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
   * @return the position in rotations of the elevator
   */
  @Logged(name = "Elevator Position Radians")
  public Angle getPosition() {
    return Rotations.of(
        (m_ElevatorMotor1.getPosition().getValue().in(Rotations)
                + m_ElevatorMotor2.getPosition().getValue().in(Rotations))
            / 2);
  }
  /**
   * logs elevator position in rotations
   * @return the position in rotations of the elevator
   */
  @Logged(name = "Elevator Position")
  public double logPosition() {
    return getPosition().in(Rotations);
  }
  /**
   * checks if the elevator is at the correct position, including if the elevator is going to the algae position
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

  public Command positionCommand(ElevatorPositions position, boolean algae) {
    return Commands.runOnce(() -> setPosition(position, algae), this)
        .until(() -> isAtPosition(position, algae));
  }

  // temp methods
  public void runSetpoint() {

    Angle rotations = Rotations.of(0);
    if (setpoint.getNumber() >= 0 && setpoint.getNumber() <= 26) {
      rotations = Rotations.of(setpoint.getNumber());
    } else {
      rotations = Rotations.of(10);
    }
    if (getPosition().in(Rotations) < rotations.in(Rotations)) {
      m_PositionRequest.Velocity = upVelocity.getNumber();
      m_PositionRequest.Acceleration = upAcceleration.getNumber();
      m_PositionRequest.Jerk = upJerk.getNumber();
    } else {
      m_PositionRequest.Velocity = downVelocity.getNumber();
      m_PositionRequest.Acceleration = downAcceleration.getNumber();
      m_PositionRequest.Jerk = downJerk.getNumber();
    }
    if (getLowerLimit() && rotations.in(Rotations) == 0) {
      m_ElevatorMotor1.setControl(
          m_CharacterizationRequest
              .withOutput(0)
              .withLimitForwardMotion(getUpperLimit())
              .withLimitReverseMotion(getLowerLimit()));
      m_ElevatorMotor2.setControl(
          m_CharacterizationRequest
              .withOutput(0)
              .withLimitForwardMotion(getUpperLimit())
              .withLimitReverseMotion(getLowerLimit()));
    } else {
      m_ElevatorMotor1.setControl(
          m_PositionRequest
              .withPosition(rotations)
              .withLimitForwardMotion(getUpperLimit())
              .withLimitReverseMotion(getLowerLimit()));
      m_ElevatorMotor2.setControl(
          m_PositionRequest
              .withPosition(rotations)
              .withLimitForwardMotion(getUpperLimit())
              .withLimitReverseMotion(getLowerLimit()));
    }
  }

  public void runSetpoint1() {
    Angle rotations = Rotations.of(0);
    if (setpoint1.getNumber() >= 0 && setpoint1.getNumber() <= 26) {
      rotations = Rotations.of(setpoint1.getNumber());
    } else {
      rotations = Rotations.of(10);
    }
    if (getPosition().in(Rotations) < rotations.in(Rotations)) {
      m_PositionRequest.Velocity = upVelocity.getNumber();
      m_PositionRequest.Acceleration = upAcceleration.getNumber();
      m_PositionRequest.Jerk = upJerk.getNumber();
    } else {
      m_PositionRequest.Velocity = downVelocity.getNumber();
      m_PositionRequest.Acceleration = downAcceleration.getNumber();
      m_PositionRequest.Jerk = downJerk.getNumber();
    }
    if (getLowerLimit() && rotations.in(Rotations) == 0) {
      m_ElevatorMotor1.setControl(
          m_CharacterizationRequest
              .withOutput(0)
              .withLimitForwardMotion(getUpperLimit())
              .withLimitReverseMotion(getLowerLimit()));
      m_ElevatorMotor2.setControl(
          m_CharacterizationRequest
              .withOutput(0)
              .withLimitForwardMotion(getUpperLimit())
              .withLimitReverseMotion(getLowerLimit()));
    } else {
      m_ElevatorMotor1.setControl(
          m_PositionRequest
              .withPosition(rotations)
              .withLimitForwardMotion(getUpperLimit())
              .withLimitReverseMotion(getLowerLimit()));
      m_ElevatorMotor2.setControl(
          m_PositionRequest
              .withPosition(rotations)
              .withLimitForwardMotion(getUpperLimit())
              .withLimitReverseMotion(getLowerLimit()));
    }
  }

  public void runSetpoint2() {
    Angle rotations = Rotations.of(0);
    if (setpoint2.getNumber() >= 0 && setpoint2.getNumber() <= 26) {
      rotations = Rotations.of(setpoint2.getNumber());
    } else {
      rotations = Rotations.of(10);
    }
    if (getPosition().in(Rotations) < rotations.in(Rotations)) {
      m_PositionRequest.Velocity = upVelocity.getNumber();
      m_PositionRequest.Acceleration = upAcceleration.getNumber();
      m_PositionRequest.Jerk = upJerk.getNumber();
    } else {
      m_PositionRequest.Velocity = downVelocity.getNumber();
      m_PositionRequest.Acceleration = downAcceleration.getNumber();
      m_PositionRequest.Jerk = downJerk.getNumber();
    }
    if (getLowerLimit() && rotations.in(Rotations) == 0) {
      m_ElevatorMotor1.setControl(
          m_CharacterizationRequest
              .withOutput(0)
              .withLimitForwardMotion(getUpperLimit())
              .withLimitReverseMotion(getLowerLimit()));
      m_ElevatorMotor2.setControl(
          m_CharacterizationRequest
              .withOutput(0)
              .withLimitForwardMotion(getUpperLimit())
              .withLimitReverseMotion(getLowerLimit()));
    } else {
      m_ElevatorMotor1.setControl(
          m_PositionRequest
              .withPosition(rotations)
              .withLimitForwardMotion(getUpperLimit())
              .withLimitReverseMotion(getLowerLimit()));
      m_ElevatorMotor2.setControl(
          m_PositionRequest
              .withPosition(rotations)
              .withLimitForwardMotion(getUpperLimit())
              .withLimitReverseMotion(getLowerLimit()));
    }
  }

  public void stopMotors() {
    m_ElevatorMotor1.setControl(m_CharacterizationRequest.withOutput(0));
    m_ElevatorMotor2.setControl(m_CharacterizationRequest.withOutput(0));
  }

  // end temp methods
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

    // only apply if one of the numbers have changed because setting the config is a
    // blocking operation
    if (elevatorKp.getNumber() != PIDConfig.kP
        || elevatorKd.getNumber() != PIDConfig.kD
        || elevatorKs.getNumber() != PIDConfig.kS
        || elevatorKg.getNumber() != PIDConfig.kG
        || elevatorKa.getNumber() != PIDConfig.kA
        || elevatorKv.getNumber() != PIDConfig.kV) {
      PIDConfig.kP = elevatorKp.getNumber();
      PIDConfig.kD = elevatorKd.getNumber();
      PIDConfig.kS = elevatorKs.getNumber();
      PIDConfig.kG = elevatorKg.getNumber();
      PIDConfig.kA = elevatorKa.getNumber();
      PIDConfig.kV = elevatorKv.getNumber();

      m_ElevatorMotor1.getConfigurator().apply(PIDConfig);
      m_ElevatorMotor2.getConfigurator().apply(PIDConfig);
      System.out.println("fff");
    }

    if (getLowerLimit()) {
      m_ElevatorMotor1.setPosition(Rotations.of(0));
      m_ElevatorMotor2.setPosition(Rotations.of(0));
    }
    if (getUpperLimit()) {
      m_ElevatorMotor1.setPosition(Rotations.of(26));
      m_ElevatorMotor2.setPosition(Rotations.of(26));
    }

    if (getPosition().in(Rotations) < m_SetpointPosition.Rotations.in(Rotations)) {
      m_PositionRequest.Velocity = upVelocity.getNumber();
      m_PositionRequest.Acceleration = upAcceleration.getNumber();
      m_PositionRequest.Jerk = upJerk.getNumber();
    } else {
      m_PositionRequest.Velocity = downVelocity.getNumber();
      m_PositionRequest.Acceleration = downAcceleration.getNumber();
      m_PositionRequest.Jerk = downJerk.getNumber();
    }
    if (getLowerLimit() && m_SetpointPosition == ElevatorPositions.HOME) {
      m_ElevatorMotor1.setControl(
          m_CharacterizationRequest
              .withOutput(0)
              .withLimitForwardMotion(getUpperLimit())
              .withLimitReverseMotion(getLowerLimit()));
      m_ElevatorMotor2.setControl(
          m_CharacterizationRequest
              .withOutput(0)
              .withLimitForwardMotion(getUpperLimit())
              .withLimitReverseMotion(getLowerLimit()));
    } else {
      m_ElevatorMotor1.setControl(
          m_PositionRequest
              .withPosition(m_SetpointPosition.Rotations)
              .withLimitForwardMotion(getUpperLimit())
              .withLimitReverseMotion(getLowerLimit()));
      m_ElevatorMotor2.setControl(
          m_PositionRequest
              .withPosition(m_SetpointPosition.Rotations)
              .withLimitForwardMotion(getUpperLimit())
              .withLimitReverseMotion(getLowerLimit()));
    }
  }
}
