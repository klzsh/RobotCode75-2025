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
import edu.wpi.first.units.measure.Distance;
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

    public final Distance inches;

    private ElevatorPositions(Distance inches) {
      this.inches = inches;
    }
  }

  @Logged(name = "Current Position", importance = Importance.INFO)
  private ElevatorPositions m_CurrentPosition = ElevatorPositions.HOME;

  private boolean m_IsAlgae = false;

  // define motors
  @Logged(name = "Left Elevator Motor", importance = Importance.DEBUG)
  private final TalonFX m_ElevatorMotor1;

  @Logged(name = "Right Elevator Motor", importance = Importance.DEBUG)
  private final TalonFX m_ElevatorMotor2;

  // sensors
  private final DigitalInput m_lowerLimitSwitch;
  private final DigitalInput m_upperLimitSwitch;
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

  public Elevator() {
    // initialize motors, using the non drivetrain CANivore bus
    m_ElevatorMotor1 = new TalonFX(elevatorMotor1CANID, superstructureCANBusName);
    m_ElevatorMotor2 = new TalonFX(elevatorMotor2CANID, superstructureCANBusName);
    // initialize sensors
    m_lowerLimitSwitch = new DigitalInput(lowerLimitPort);
    m_upperLimitSwitch = new DigitalInput(upperLimitPort);
    // m_distanceSensor = new Counter(distanceSensorPort);
    // initialize control requests
    m_PositionRequest = new DynamicMotionMagicTorqueCurrentFOC(0, 0, 0, 0);
    m_CharacterizationRequest = new TorqueCurrentFOC(Amps.of(0));
    // configure motors with correct inverts
    m_ElevatorMotor1.getConfigurator().apply(getElevatorMotorConfig());
    m_ElevatorMotor2.getConfigurator().apply(getElevatorMotorConfig());
    // set the position of the elevator
    m_ElevatorMotor1.setPosition(inchesToRotations(ElevatorPositions.HOME.inches));
    m_ElevatorMotor2.setPosition(inchesToRotations(ElevatorPositions.HOME.inches));

    // sets the default values for the tunable numbers
    upVelocity = new TunableNumber("Elevator/UpVelocity", MotionMagicProfileUp[0]);
    upAcceleration = new TunableNumber("Elevator/UpAcceleration", MotionMagicProfileUp[1]);
    upJerk = new TunableNumber("Elevator/UpJerk", MotionMagicProfileUp[2]);

    downVelocity = new TunableNumber("Elevator/DownVelocity", MotionMagicProfileDown[0]);
    downAcceleration = new TunableNumber("Elevator/DownAcceleration", MotionMagicProfileDown[1]);
    downJerk = new TunableNumber("Elevator/DownJerk", MotionMagicProfileDown[2]);

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
    m_CurrentPosition = position;
    m_IsAlgae = isAlgae;
  }

  public boolean isAtPosition(ElevatorPositions position, boolean isAlgae) {
    if (position == ElevatorPositions.HOME) return getLowerLimit();
    if (position == ElevatorPositions.L4) return getUpperLimit();
    double currentPosition =
        rotationsToInches(m_ElevatorMotor1.getPosition().getValue()).in(Inches);
    double algaeOffset =
        (isAlgae && (position == ElevatorPositions.L2 || position == ElevatorPositions.L3))
            ? algaeRemovalOffset.in(Inches)
            : 0;
    return MathUtil.applyDeadband(
            currentPosition - position.inches.in(Inches) - algaeOffset, deadband.in(Inches))
        == 0.0;
  }

  /**
   * Raw rotations to inches. Does not take into account offsets
   *
   * @param rotations the rotations to convert to
   * @return inches the elevator has moved
   */
  private Distance rotationsToInches(Angle rotations) {
    return Inches.of(rotations.in(Rotations) * inchesPerRotation.in(Inches));
  }

  private Angle inchesToRotations(Distance inches) {
    return Rotations.of(inches.in(Inches) / inchesPerRotation.in(Inches));
  }

  public Command positionCommand(ElevatorPositions position, boolean algae) {
    return Commands.runOnce(() -> setPosition(position, algae), this)
        .until(() -> isAtPosition(position, algae));
  }

  // temp methods
  public void runSetpoint1() {
    m_PositionRequest.Velocity = upVelocity.getNumber();
    m_PositionRequest.Acceleration = upAcceleration.getNumber();
    m_PositionRequest.Jerk = upJerk.getNumber();
    m_ElevatorMotor1.setControl(
        m_PositionRequest
            .withPosition(17)
            .withLimitForwardMotion(getUpperLimit())
            .withLimitReverseMotion(getLowerLimit()));
    m_ElevatorMotor2.setControl(
        m_PositionRequest
            .withPosition(17)
            .withLimitForwardMotion(getUpperLimit())
            .withLimitReverseMotion(getLowerLimit()));
  }

  public void runSetpoint2() {
    m_ElevatorMotor1.setControl(m_CharacterizationRequest.withOutput(15));
    m_ElevatorMotor2.setControl(m_CharacterizationRequest.withOutput(15));
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
    return !m_lowerLimitSwitch.get();
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
      m_ElevatorMotor1.setPosition(Rotations.of(25));
      m_ElevatorMotor2.setPosition(Rotations.of(25));
    }
    double target = 0;
    if (!isAtPosition(m_CurrentPosition, m_IsAlgae)) {
      Distance currentPosition = rotationsToInches(m_ElevatorMotor1.getPosition().getValue());
      double algaeOffset =
          (m_IsAlgae
                  && (m_CurrentPosition == ElevatorPositions.L2
                      || m_CurrentPosition == ElevatorPositions.L3))
              ? algaeRemovalOffset.in(Inches)
              : 0;
      target = m_CurrentPosition.inches.in(Inches) + algaeOffset;

      if (currentPosition.in(Inches) < target) {
        m_PositionRequest.Velocity = upVelocity.getNumber();
        m_PositionRequest.Acceleration = upAcceleration.getNumber();
        m_PositionRequest.Jerk = upJerk.getNumber();
      } else {
        m_PositionRequest.Velocity = downVelocity.getNumber();
        m_PositionRequest.Acceleration = downAcceleration.getNumber();
        m_PositionRequest.Jerk = downJerk.getNumber();
      }
    }
    // uncomment when done profiling elevator
      // m_ElevatorMotor1.setControl(
      //     m_PositionRequest
      //         .withPosition(inchesToRotations(Inches.of(target)))
      //         .withLimitForwardMotion(getUpperLimit())
      //         .withLimitReverseMotion(getLowerLimit()));
      // m_ElevatorMotor2.setControl(
      //     m_PositionRequest
      //         .withPosition(inchesToRotations(Inches.of(target)))
      //         .withLimitForwardMotion(getUpperLimit())
      //         .withLimitReverseMotion(getLowerLimit()));
  }
}
