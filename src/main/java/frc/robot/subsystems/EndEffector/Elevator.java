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
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.MutDistance;
import edu.wpi.first.units.measure.MutLinearVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Counter;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Mechanism;
import frc.lib.dashboard.TunableNumber;
import frc.robot.Constants.ElevatorConstants;

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
  @Logged(name = "Left Elevator Motor", importance = Importance.INFO)
  private final TalonFX m_ElevatorMotor1;

  @Logged(name = "Right Elevator Motor", importance = Importance.INFO)
  private final TalonFX m_ElevatorMotor2;

  // sensors
  private final DigitalInput m_lowerLimitSwitch;
  private final DigitalInput m_upperLimitSwitch;
  private final Counter m_distanceSensor;

  // define control requests
  private final DynamicMotionMagicTorqueCurrentFOC m_PositionRequest;
  private final TorqueCurrentFOC m_CharacterizationRequest;

  // motor characterization stuff
  private final SysIdRoutine m_Routine;

  private final MutVoltage m_appliedVoltage = Volts.mutable(0);
  private final MutDistance m_distance = Inches.mutable(0);
  private final MutLinearVelocity m_velocity = InchesPerSecond.mutable(0);

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

  public Elevator() {
    // initialize motors, using the non drivetrain CANivore bus
    m_ElevatorMotor1 = new TalonFX(elevatorMotor1CANID, superstructureCANBusName);
    m_ElevatorMotor2 = new TalonFX(elevatorMotor2CANID, superstructureCANBusName);
    // initialize sensors
    m_lowerLimitSwitch = new DigitalInput(lowerLimitPort);
    m_upperLimitSwitch = new DigitalInput(upperLimitPort);
    m_distanceSensor = new Counter(distanceSensorPort);
    // initialize control requests
    m_PositionRequest = new DynamicMotionMagicTorqueCurrentFOC(0, 0, 0, 0);
    m_CharacterizationRequest = new TorqueCurrentFOC(Amps.of(0));
    // configure motors with correct inverts
    m_ElevatorMotor1.getConfigurator().apply(getElevatorMotorConfig());
    m_ElevatorMotor2.getConfigurator().apply(getElevatorMotorConfig());
    // set the position of the elevator
    m_ElevatorMotor1.setPosition(inchesToRotations(ElevatorPositions.HOME.inches));
    m_ElevatorMotor2.setPosition(inchesToRotations(ElevatorPositions.HOME.inches));

    // set up sysid
    m_Routine =
        new SysIdRoutine(
            // 12 amps per second                   7 amps per step        timeout
            new Config(Volts.of(12).per(Seconds), Volts.of(7), Seconds.of(5)),
            new Mechanism(this::characterizeElevator, this::logMotors, this));

    // sets the default values for the tunable numbers
    upVelocity = new TunableNumber("/Elevator/UpVelocity", MotionMagicProfileUp[0]);
    upAcceleration = new TunableNumber("/Elevator/UpAcceleration", MotionMagicProfileUp[1]);
    upJerk = new TunableNumber("/Elevator/UpJerk", MotionMagicProfileUp[2]);

    downVelocity = new TunableNumber("/Elevator/DowbVelocity", MotionMagicProfileDown[0]);
    downAcceleration = new TunableNumber("/Elevator/DownAcceleration", MotionMagicProfileDown[1]);
    downJerk = new TunableNumber("/Elevator/DownJerk", MotionMagicProfileDown[2]);

    PIDConfig.withKA(kA)
        .withKS(kS)
        .withKV(kV)
        .withKG(kG)
        .withKP(kP)
        .withKD(kD)
        .withGravityType(GravityTypeValue.Elevator_Static)
        .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseVelocitySign);

    elevatorKp = new TunableNumber("/Elevator/kP", kP);
    elevatorKd = new TunableNumber("/Elevator/kD", kD);
    elevatorKg = new TunableNumber("/Elevator/kG", kG);
    elevatorKs = new TunableNumber("/Elevator/kS", kS);

    m_CharacterizationRequest.UpdateFreqHz = 0;
    m_CharacterizationRequest.UseTimesync = true;

    m_PositionRequest.UpdateFreqHz = 0;
    m_PositionRequest.UseTimesync = true;
  }

  /**
   * This function is a bit confusing due to how SysID works. SysID works with volts, but the
   * concept of characterizing the kS, kV, and kA are all the same between current and voltage
   * control. So what needs to happen is this function takes in a "voltage" value, but we set up the
   * {@link edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config} rate parameters to be more
   * representitive of current values. It is a jank workaround to WPILib not supporting an extremely
   * small subset of FRC teams
   *
   * @param current
   */
  public void characterizeElevator(Voltage current) {
    // absolutely cursed
    m_ElevatorMotor1.setControl(m_CharacterizationRequest.withOutput(current.in(Volts)));
    m_ElevatorMotor2.setControl(m_CharacterizationRequest.withOutput(current.in(Volts)));
  }

  public void logMotors(SysIdRoutineLog log) {
    log.motor("elevator1")
        .voltage(
            m_appliedVoltage.mut_replace(
                m_ElevatorMotor1.getMotorVoltage().getValue().in(Volts), Volts))
        .linearPosition(
            m_distance.mut_replace(
                rotationsToInches(m_ElevatorMotor1.getPosition().getValue()).in(Inches), Inches))
        .linearVelocity(
            m_velocity.mut_replace(
                rotationsPerSecondToInchesPerSecond(m_ElevatorMotor1.getVelocity().getValue())
                    .in(InchesPerSecond),
                InchesPerSecond));
    log.motor("elevator2")
        .voltage(
            m_appliedVoltage.mut_replace(
                m_ElevatorMotor2.getMotorVoltage().getValue().in(Volts), Volts))
        .linearPosition(
            m_distance.mut_replace(
                rotationsToInches(m_ElevatorMotor2.getPosition().getValue()).in(Inches), Inches))
        .linearVelocity(
            m_velocity.mut_replace(
                rotationsPerSecondToInchesPerSecond(m_ElevatorMotor2.getVelocity().getValue())
                    .in(InchesPerSecond),
                InchesPerSecond));
  }

  /**
   * this function should account for whether the elevator is going up or down and modify parameters
   * like velocity and acceleration
   *
   * @param position the position to set the elevator to
   */
  public void setPosition(ElevatorPositions position, boolean isAlgae) {
    m_CurrentPosition = position;
    m_IsAlgae = isAlgae;
  }

  public boolean isAtPosition(ElevatorPositions position, boolean isAlgae) {
    if (position == ElevatorPositions.HOME) return m_lowerLimitSwitch.get();
    if (position == ElevatorPositions.L4) return m_upperLimitSwitch.get();
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

  private Distance rotationsToInches(Angle rotations) {
    return Inches.of(rotations.in(Rotations) * inchesPerRotation.in(Inches));
  }

  private Angle inchesToRotations(Distance inches) {
    return Rotations.of(inches.in(Inches) / inchesPerRotation.in(Inches));
  }

  private LinearVelocity rotationsPerSecondToInchesPerSecond(AngularVelocity rotations) {
    return InchesPerSecond.of(rotations.in(RotationsPerSecond) * inchesPerRotation.in(Inches));
  }

  private AngularVelocity inchesPerSecondToRotationsPerSecond(LinearVelocity inches) {
    return RotationsPerSecond.of(inches.in(InchesPerSecond) / inchesPerRotation.in(Inches));
  }

  public Command quasistaticForward() {
    return m_Routine.quasistatic(SysIdRoutine.Direction.kForward);
  }

  public Command quasistaticReverse() {
    return m_Routine.quasistatic(SysIdRoutine.Direction.kReverse);
  }

  public Command dynamicForward() {
    return m_Routine.dynamic(SysIdRoutine.Direction.kForward);
  }

  public Command dynamicReverse() {
    return m_Routine.dynamic(SysIdRoutine.Direction.kReverse);
  }

  public Command positionCommand(ElevatorPositions position, boolean algae) {
    return Commands.runOnce(() -> setPosition(position, algae), this)
        .until(() -> isAtPosition(position, algae));
  }

  @Override
  public void periodic() {

    // only apply if one of the numbers are not equal to the others because setting the config is a
    // blocking operation
    if (elevatorKp.getNumber() != PIDConfig.kP
        || elevatorKd.getNumber() != PIDConfig.kD
        || elevatorKs.getNumber() != PIDConfig.kS
        || elevatorKg.getNumber() != PIDConfig.kG) {
      PIDConfig.kP = elevatorKp.getNumber();
      PIDConfig.kD = elevatorKd.getNumber();
      PIDConfig.kS = elevatorKs.getNumber();
      PIDConfig.kG = elevatorKg.getNumber();

      m_ElevatorMotor1.getConfigurator().apply(PIDConfig);
      m_ElevatorMotor2.getConfigurator().apply(PIDConfig);
    }

    if (m_lowerLimitSwitch.get()) {
      m_ElevatorMotor1.setPosition(inchesToRotations(ElevatorPositions.HOME.inches));
      m_ElevatorMotor2.setPosition(inchesToRotations(ElevatorPositions.HOME.inches));
    }
    if (m_upperLimitSwitch.get()) {
      m_ElevatorMotor1.setPosition(inchesToRotations(ElevatorPositions.L4.inches));
      m_ElevatorMotor2.setPosition(inchesToRotations(ElevatorPositions.L4.inches));
    }

    double distance =
        Millimeter.of(m_distanceSensor.getDistance()).in(Inches)
            + motorCounterOffset.in(Inches); // inches
    double deviation =
        Math.abs(
            distance
                - rotationsToInches(m_ElevatorMotor1.getPosition().getValue())
                    .in(Inches)); // inches
    if (deviation > ElevatorConstants.maxDeviation.in(Inches)) {
      m_ElevatorMotor1.setPosition(inchesToRotations(Inches.of(distance)));
      m_ElevatorMotor2.setPosition(inchesToRotations(Inches.of(distance)));
    }

    if (!isAtPosition(m_CurrentPosition, m_IsAlgae)) {
      Distance currentPosition = rotationsToInches(m_ElevatorMotor1.getPosition().getValue());
      double algaeOffset =
          (m_IsAlgae
                  && (m_CurrentPosition == ElevatorPositions.L2
                      || m_CurrentPosition == ElevatorPositions.L3))
              ? algaeRemovalOffset.in(Inches)
              : 0;
      double target = m_CurrentPosition.inches.in(Inches) + algaeOffset;

      if (currentPosition.in(Inches) < target) {
        if (m_upperLimitSwitch.get()) return;
        m_PositionRequest.Velocity = upVelocity.getNumber();
        m_PositionRequest.Acceleration = upAcceleration.getNumber();
        m_PositionRequest.Jerk = upJerk.getNumber();
      } else {
        if (m_lowerLimitSwitch.get()) return;
        m_PositionRequest.Velocity = downVelocity.getNumber();
        m_PositionRequest.Acceleration = downAcceleration.getNumber();
        m_PositionRequest.Jerk = downJerk.getNumber();
      }

      m_ElevatorMotor1.setControl(
          m_PositionRequest.withPosition(inchesToRotations(Inches.of(target))));
      m_ElevatorMotor2.setControl(
          m_PositionRequest.withPosition(inchesToRotations(Inches.of(target))));
    }
  }
}
