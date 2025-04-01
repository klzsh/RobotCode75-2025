// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.EndGame;

import static edu.wpi.first.units.Units.*;
import static frc.robot.Constants.ClimberConstants.*;
import static frc.robot.Constants.ClimberConstants.MotorConfigs.*;
import static frc.robot.Constants.RobotConstants.superstructureCANBusName;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.controls.MotionMagicExpoTorqueCurrentFOC;
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
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;

@Logged(name = "Climber", strategy = Strategy.OPT_IN, importance = Importance.CRITICAL)
public class Climber extends SubsystemBase {

  // @Logged(name = "Climber Motor 1", importance = Importance.DEBUG)
  private final TalonFX m_ClimberMotor1;

  // @Logged(name = "Climber Motor 2", importance = Importance.DEBUG)
  private final TalonFX m_ClimberMotor2;

  // private final DutyCycleEncoder m_ClimberEncoder;

  // private Slot0Configs PIDConfig = new Slot0Configs();
  // private final TunableNumber climberKp;
  // private final TunableNumber climberKd;
  // private final TunableNumber climberKg;
  // private final TunableNumber climberKs;
  // private final TunableNumber climberKa;
  // private final TunableNumber climberKv;

  // private MotionMagicConfigs MMConfig = new MotionMagicConfigs();
  // private final TunableNumber climberMMCruiseVelocity;
  // private final TunableNumber climberMMCruiseAcceleration;
  // private final TunableNumber climberMMKv;
  // private final TunableNumber climberMMKa;

  // private final TunableNumber zeroPoint;
  // private final TunableNumber retractSetpoint;
  // private final TunableNumber extendSetpoint;

  private final MotionMagicExpoTorqueCurrentFOC m_PositionRequest;

  private final TorqueCurrentFOC m_TestRequest;

  private final DigitalInput m_ClimberLimitSwitch;

  @Logged(name = "at position", importance = Importance.CRITICAL)
  private boolean atPosition = false;

  public Climber() {
    m_ClimberMotor1 = new TalonFX(ClimberConstants.climberMotor1CANID, superstructureCANBusName);
    m_ClimberMotor2 = new TalonFX(ClimberConstants.climberMotor2CANID, superstructureCANBusName);

    // m_ClimberEncoder = new DutyCycleEncoder(climberEncoderPort, 1, 0.855);

    // climberMMCruiseVelocity =
    //     new TunableNumber("Climber/Cruise Velocity", motionMagicCruiseVelocity);
    // climberMMCruiseAcceleration =
    //     new TunableNumber("Climber/Cruise Acceleration", motionMagicCruiseAcceleration);
    // climberMMKv = new TunableNumber("Climber/MM kV", motionMagickV);
    // climberMMKa = new TunableNumber("Climber/MM kA", motionMagickA);

    m_PositionRequest = new MotionMagicExpoTorqueCurrentFOC(0);
    m_TestRequest = new TorqueCurrentFOC(Amps.of(0));

    m_ClimberLimitSwitch = new DigitalInput(8);

    // retractSetpoint = new TunableNumber("Climber/Retract Setpoint", climbPosition.in(Rotations));
    // extendSetpoint = new TunableNumber("Climber/Extend Setpoint",
    // climbExtendPosition.in(Rotations));

    // PIDConfig.withKA(kA)
    //     .withKS(kS)
    //     .withKV(kV)
    //     .withKG(kG)
    //     .withKP(kP)
    //     .withKD(kD)
    //     .withGravityType(GravityTypeValue.Arm_Cosine)
    //     .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseVelocitySign);

    // climberKp = new TunableNumber("Climber/kP", kP);
    // climberKd = new TunableNumber("Climber/kD", kD);
    // climberKg = new TunableNumber("Climber/kG", kG);
    // climberKs = new TunableNumber("Climber/kS", kS);
    // climberKa = new TunableNumber("Climber/kA", kA);
    // climberKv = new TunableNumber("Climber/kV", kV);

    m_ClimberMotor1.getConfigurator().apply(getClimberMotorConfig());
    m_ClimberMotor2.getConfigurator().apply(getClimberMotorConfig());

    // zeroPoint = new TunableNumber("Climber/Encoder Offset", 0);
    Timer.delay(5);
    resetPosition();
  }

  @Logged(name = "Climber Limit Switch", importance = Importance.CRITICAL)
  public boolean getLimitSwitch() {
    return m_ClimberLimitSwitch.get();
  }

  public double absoluteEncoderToRotations(double x) {
    return 132.2772 * Math.sin(3.36922 * x) + 11.17;
  }

  public void setPositionRequest(Angle position) {
    if ((getLimitSwitch() || atPosition(position))
        && position.in(Rotations) <= getPositionRotations()) {
      m_ClimberMotor1.setControl(m_TestRequest.withOutput(0));
      m_ClimberMotor2.setControl(m_TestRequest.withOutput(0));
      atPosition = true;
    } else {
      m_ClimberMotor1.setControl(
          m_PositionRequest.withPosition(position).withLimitReverseMotion(getLimitSwitch()));
      m_ClimberMotor2.setControl(
          m_PositionRequest.withPosition(position).withLimitReverseMotion(getLimitSwitch()));
      atPosition = false;
    }
  }

  // @Logged(name = "Climber Absolute Encoder", importance = Importance.CRITICAL)
  // public double getAbsolutePosition() {
  //   return m_ClimberEncoder.get();
  // }

  @Logged(name = "Climber Position", importance = Importance.CRITICAL)
  public double getPositionRotations() {
    return getPosition().in(Rotations);
  }

  public Angle getPosition() {
    Measure<AngleUnit> motor1Position =
        BaseStatusSignal.getLatencyCompensatedValue(
            m_ClimberMotor1.getPosition(true), m_ClimberMotor1.getVelocity(true));
    Measure<AngleUnit> motor2Position =
        BaseStatusSignal.getLatencyCompensatedValue(
            m_ClimberMotor2.getPosition(true), m_ClimberMotor2.getVelocity(true));

    return Rotations.of((motor1Position.in(Rotations) + motor2Position.in(Rotations)) / 2);
  }

  public boolean atPosition(Angle position) {
    return Math.abs(getPosition().in(Rotations) - position.in(Rotations))
        < ClimberConstants.climbDeadband;
  }

  public void runCurrent(double current) {
    current = MathUtil.applyDeadband(current, 5);

    if (getLimitSwitch() && current < 0) {
      current = 0;
    }

    m_ClimberMotor1.setControl(m_TestRequest.withOutput(current));
    m_ClimberMotor2.setControl(m_TestRequest.withOutput(current));
  }

  public void resetPosition() {
    // m_ClimberMotor1.setPosition(absoluteEncoderToRotations(getAbsolutePosition()));
    // m_ClimberMotor2.setPosition(absoluteEncoderToRotations(getAbsolutePosition()));
    // ! what is this for?
    if (getLimitSwitch()) {
      m_ClimberMotor1.setPosition(0);
      m_ClimberMotor2.setPosition(0);
    }
  }

  @Override
  public void periodic() {
    if (getLimitSwitch() && Math.abs(getPosition().in(Rotations)) >= 0.1) {
      resetPosition();
    }
    SmartDashboard.putBoolean("Climber Limit", getLimitSwitch());
    // if (climberKp.getNumber() != PIDConfig.kP
    //     || climberKd.getNumber() != PIDConfig.kD
    //     || climberKs.getNumber() != PIDConfig.kS
    //     || climberKg.getNumber() != PIDConfig.kG
    //     || climberKa.getNumber() != PIDConfig.kA
    //     || climberKv.getNumber() != PIDConfig.kV) {

    //   PIDConfig.kP = climberKp.getNumber();
    //   PIDConfig.kD = climberKd.getNumber();
    //   PIDConfig.kS = climberKs.getNumber();
    //   PIDConfig.kG = climberKg.getNumber();
    //   PIDConfig.kA = climberKa.getNumber();
    //   PIDConfig.kV = climberKv.getNumber();

    //   m_ClimberMotor1.getConfigurator().apply(PIDConfig);
    //   m_ClimberMotor2.getConfigurator().apply(PIDConfig);
    // }
    // if (climberMMCruiseVelocity.getNumber() != MMConfig.MotionMagicCruiseVelocity
    //     || climberMMCruiseAcceleration.getNumber() != MMConfig.MotionMagicAcceleration
    //     || climberMMKv.getNumber() != MMConfig.MotionMagicExpo_kV
    //     || climberMMKa.getNumber() != MMConfig.MotionMagicExpo_kA) {

    //   MMConfig.MotionMagicCruiseVelocity = climberMMCruiseVelocity.getNumber();
    //   MMConfig.MotionMagicAcceleration = climberMMCruiseAcceleration.getNumber();
    //   MMConfig.MotionMagicExpo_kV = climberMMKv.getNumber();
    //   MMConfig.MotionMagicExpo_kA = climberMMKa.getNumber();

    //   m_ClimberMotor1.getConfigurator().apply(MMConfig);
    //   m_ClimberMotor2.getConfigurator().apply(MMConfig);
    // }
  }
}
