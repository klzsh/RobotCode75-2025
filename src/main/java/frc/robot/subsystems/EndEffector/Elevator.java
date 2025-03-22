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
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Importance;
import edu.wpi.first.epilogue.Logged.Strategy;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.dashboard.TunableNumber;

/*
 * Cascading elevator driven by 2 Kraken X60s
 * 2 stage WCP GreyT elevator
 * Uses motion magic to honor a target \ and acceleration
 *  Motion magic is used to prevent the elevator from destroying itself by moving too fast
 *
 */
@Logged(strategy = Strategy.OPT_IN, name = "Elevator", importance = Importance.CRITICAL)
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

  @Logged(name = "Current Position", importance = Importance.CRITICAL)
  private ElevatorPositions m_SetpointPosition = ElevatorPositions.HOME;

  // @Logged
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

  // define control requests
  private final DynamicMotionMagicTorqueCurrentFOC m_PositionRequest;
  private final TorqueCurrentFOC m_CharacterizationRequest;

  // private final TunableNumber mmVelocityUp;
  // private final TunableNumber mmAccelerationUp;
  // private final TunableNumber mmJerkUp;

  // private final TunableNumber mmVelocityDown;
  // private final TunableNumber mmAccelerationDown;
  // private final TunableNumber mmJerkDown;

  // private final TunableNumber elevatorKp;
  // private final TunableNumber elevatorKi;
  // private final TunableNumber elevatorKd;
  // private final TunableNumber elevatorKs;
  // private final TunableNumber elevatorKa;
  // private final TunableNumber elevatorKv;
  // private final TunableNumber elevatorKg;
  // private Slot0Configs config;

  private final TunableNumber l2Height;
  private final TunableNumber l3Height;

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

    l2Height = new TunableNumber("Elevator/L2 Height", l2Position.in(Rotations));
    l3Height = new TunableNumber("Elevator/L3 Height", l3Position.in(Rotations));

    // mmVelocityUp = new TunableNumber("Elevator/MM Velocity Up", MotionMagicProfileUp[0]);
    // mmAccelerationUp = new TunableNumber("Elevator/MM Accleration Up", MotionMagicProfileUp[1]);
    // mmJerkUp = new TunableNumber("Elevator/MM Jerk Up", MotionMagicProfileUp[2]);

    // mmVelocityDown = new TunableNumber("Elevator/MM Velocity Down", MotionMagicProfileDown[0]);
    // mmAccelerationDown =
    //     new TunableNumber("Elevator/MM Acceleration Down", MotionMagicProfileDown[1]);
    // mmJerkDown = new TunableNumber("Elevator/MM Jerk Down", MotionMagicProfileDown[2]);

    // elevatorKp = new TunableNumber("Elevator/kP", kP);
    // elevatorKi = new TunableNumber("Elevator/kI", kI);
    // elevatorKd = new TunableNumber("Elevator/kD", kD);
    // elevatorKs = new TunableNumber("Elevator/kS", kS);
    // elevatorKa = new TunableNumber("Elevator/kA", kA);
    // elevatorKv = new TunableNumber("Elevator/kV", kV);
    // elevatorKg = new TunableNumber("Elevator/kG", kG);

    // config =
    //     new Slot0Configs()
    //         .withKP(kP)
    //         .withKI(kI)
    //         .withKD(kD)
    //         .withKS(kS)
    //         .withKA(kA)
    //         .withKV(kV)
    //         .withKG(kG)
    //         .withGravityType(GravityTypeValue.Elevator_Static)
    //         .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseVelocitySign);
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

  public ElevatorPositions getState() {
    return m_SetpointPosition;
  }

  /**
   * logs elevator position in rotations
   *
   * @return the position in rotations of the elevator
   */
  @Logged(name = "Elevator Position", importance = Importance.CRITICAL)
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
            // ? AlgaeOffsetPrePickupRotations.getNumber()
            : 0;
    return MathUtil.applyDeadband(
            currentPosition - position.Rotations.in(Rotations) - algaeOffset,
            deadband.in(Rotations))
        == 0.0;
  }

  public boolean isBelowPosition(ElevatorPositions position, boolean isAlgae) {
    if (position == ElevatorPositions.HOME) {
      return getLowerLimit();
    }
    double currentPosition = getPosition().in(Rotations);
    double algaeOffset =
        (isAlgae && (position == ElevatorPositions.L2 || position == ElevatorPositions.L3))
            ? algaeRemovalOffset.in(Rotations)
            // ? AlgaeOffsetPrePickupRotations.getNumber()
            : 0;
    currentPosition += algaeOffset;
    return currentPosition <= position.Rotations.in(Rotations);
  }

  @Logged(name = "Upper Limit", importance = Importance.CRITICAL)
  public boolean getUpperLimit() {
    return !m_upperLimitSwitch.get();
  }

  @Logged(name = "Lower Limit", importance = Importance.CRITICAL)
  public boolean getLowerLimit() {
    return !m_lowerLimitSwitch.get() || !m_backupLimitSwitch.get();
  }
  @Logged(name = "Lower Limit One", importance = Importance.CRITICAL)
  public boolean getLowerLimitOne(){
    return !m_lowerLimitSwitch.get();
  }
  @Logged(name = "Lower Limit Two", importance = Importance.CRITICAL)
  public boolean getLowerLimitTwo(){
    return !m_backupLimitSwitch.get();
  }

  @Override
  public void periodic() {
    if (getLowerLimit()
        && (getPosition().in(Rotations) >= 0.1 || getPosition().in(Rotations) <= -0.1)) {
      m_ElevatorMotor1.setPosition(Rotations.of(0));
      m_ElevatorMotor2.setPosition(Rotations.of(0));
      // System.out.println("Called");
    }
    // if (getUpperLimit()) {
    //   m_ElevatorMotor1.setPosition(Rotations.of(26));
    //   m_ElevatorMotor2.setPosition(Rotations.of(26));
    // }
    double currentPosition = m_SetpointPosition.Rotations.in(Rotations);
    if (m_SetpointPosition == ElevatorPositions.L2) {
      currentPosition = l2Height.getNumber();
    }
    if (m_SetpointPosition == ElevatorPositions.L3) {
      currentPosition = l3Height.getNumber();
    }

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

      // m_PositionRequest.Velocity = mmVelocityUp.getNumber();
      // m_PositionRequest.Acceleration = mmAccelerationUp.getNumber();
      // m_PositionRequest.Jerk = mmJerkUp.getNumber();
    } else {
      m_PositionRequest.Velocity = MotionMagicProfileDown[0];
      m_PositionRequest.Acceleration = MotionMagicProfileDown[1];
      m_PositionRequest.Jerk = MotionMagicProfileDown[2];
      // m_PositionRequest.Velocity = mmVelocityDown.getNumber();
      // m_PositionRequest.Acceleration = mmAccelerationDown.getNumber();
      // m_PositionRequest.Jerk = mmJerkDown.getNumber();
    }

    // if (config.kP != elevatorKp.getNumber()
    //     || config.kI != elevatorKi.getNumber()
    //     || config.kD != elevatorKd.getNumber()
    //     || config.kS != elevatorKs.getNumber()
    //     || config.kA != elevatorKa.getNumber()
    //     || config.kV != elevatorKv.getNumber()
    //     || config.kG != elevatorKg.getNumber()) {
    //   config.kP = elevatorKp.getNumber();
    //   config.kI = elevatorKi.getNumber();
    //   config.kD = elevatorKd.getNumber();
    //   config.kS = elevatorKs.getNumber();
    //   config.kA = elevatorKa.getNumber();
    //   config.kV = elevatorKv.getNumber();
    //   config.kG = elevatorKg.getNumber();

    //   m_ElevatorMotor1.getConfigurator().apply(config);
    //   m_ElevatorMotor2.getConfigurator().apply(config);
    // }

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
