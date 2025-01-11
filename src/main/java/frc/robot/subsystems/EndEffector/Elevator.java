// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.EndEffector;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.Constants.ElevatorConstants.*;
import static frc.robot.Constants.HardwareConstants.*;

import com.ctre.phoenix6.controls.DynamicMotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Mechanism;
import frc.robot.Constants.HardwareConstants;

/*
 * Cascading elevator driven by 2 Kraken X60s
 * 2 stage WCP GreyT elevator
 * Uses motion magic to honor a target cruise velocity and acceleration
 *  Motion magic is used to prevent the elevator from destroying itself by moving too fast
 *
 */
public class Elevator extends SubsystemBase {

  // we only have a certain number of states the elevator will be in at any given time
  public enum ElevatorPositions {
    L1(l1Position),
    L2(l2Position),
    L3(l3Position),
    L4(l4Position),
    HOME(homePosition),
    PROCESSOR(processorPosition),
    HPStation(HPStationPosition);

    public final Distance rotations;

    private ElevatorPositions(Distance rotations) {
      this.rotations = rotations;
    }
  }

  // define motors
  private final TalonFX m_ElevatorMotor1;
  private final TalonFX m_ElevatorMotor2;

  // define control requests
  // TODO: check whether or not to use dynamic or expo motion magic (leaning towards dynamic)
  private final DynamicMotionMagicTorqueCurrentFOC m_PositionRequest;
  private final TorqueCurrentFOC m_CharacterizationRequest;

  // motor characterization stuff
  private final SysIdRoutine m_Routine;

  public Elevator() {
    // initialize motors, using the non drivetrain CANivore bus
    m_ElevatorMotor1 = new TalonFX(elevatorMotor1CANID, superstructureCANBusName);
    m_ElevatorMotor2 = new TalonFX(elevatorMotor2CANID, superstructureCANBusName);
    // initialize control requests
    m_PositionRequest = new DynamicMotionMagicTorqueCurrentFOC(0, 0, 0, 0);
    m_CharacterizationRequest = new TorqueCurrentFOC(Amps.of(0));
    // configure motors with correct inverts
    m_ElevatorMotor1
        .getConfigurator()
        .apply(HardwareConstants.Elevator.getElevatorMotorConfig(false));
    m_ElevatorMotor2
        .getConfigurator()
        .apply(HardwareConstants.Elevator.getElevatorMotorConfig(true));
    // set the position of the elevator
    // TODO: determine where the "zero" point of the elevator is (floor or fully retracted elevator)
    m_ElevatorMotor1.setPosition(0);
    m_ElevatorMotor2.setPosition(0);

    // set up sysid
    m_Routine =
        new SysIdRoutine(
            new Config(Volts.of(10).per(Seconds), Volts.of(12), Seconds.of(4)),
            new Mechanism(this::characterizeElevator, null, this));
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
  public void characterizeElevator(Voltage current) {}

  // TODO: make sure this function signature makes sense. DISCUSS ON 1/11/25
  /**
   * this function should account for whether the elevator is going up or down and modify parameters
   * like velocity and acceleration
   *
   * @param positions the position to set the elevator to
   */
  public void setPosition(ElevatorPositions positions) {}

  private Distance rotationsToInches(Angle rotations) {
    return Inches.of(rotations.in(Rotations) * inchesPerRotation.in(Inches));
  }

  private Angle inchesToRotations(Distance inches) {
    return Rotations.of(inches.in(Inches) / inchesPerRotation.in(Inches));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
