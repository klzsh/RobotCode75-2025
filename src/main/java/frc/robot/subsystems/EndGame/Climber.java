// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.EndGame;

import static edu.wpi.first.units.Units.*;
import static frc.robot.Constants.ClimberConstants.*;
import static frc.robot.Constants.ClimberConstants.MotorConfigs.*;
import static frc.robot.Constants.RobotConstants.superstructureCANBusName;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.MotionMagicExpoTorqueCurrentFOC;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Strategy;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.lib.dashboard.TunableNumber;
import frc.robot.Constants.ClimberConstants;

@Logged(name = "Climber", strategy = Strategy.OPT_IN)
public class Climber extends SubsystemBase {

  public static enum ClimberPositions {
    DEFAULT,
    CLIMB
  }

  // @Logged(name = "Climber Motor 1", importance = Importance.DEBUG)
  private final TalonFX m_ClimberMotor1;

  // @Logged(name = "Climber Motor 2", importance = Importance.DEBUG)
  private final TalonFX m_ClimberMotor2;

  private final DutyCycleEncoder m_ClimberEncoder;

  // @Logged(name = "Climber State", importance = Importance.CRITICAL)
  private ClimberPositions m_ClimberState = ClimberPositions.DEFAULT;

  private Slot0Configs PIDConfig = new Slot0Configs();
  private final TunableNumber climberKp;
  private final TunableNumber climberKd;
  private final TunableNumber climberKg;
  private final TunableNumber climberKs;
  private final TunableNumber climberKa;
  private final TunableNumber climberKv;

  private MotionMagicConfigs MMConfig = new MotionMagicConfigs();
  private final TunableNumber climberMMCruiseVelocity;
  private final TunableNumber climberMMCruiseAcceleration;
  private final TunableNumber climberMMKv;
  private final TunableNumber climberMMKa;

  private final TunableNumber zeroPoint;

  private final MotionMagicExpoTorqueCurrentFOC m_PositionRequest;

  private final TorqueCurrentFOC m_TestRequest;

  public Climber() {
    m_ClimberMotor1 = new TalonFX(ClimberConstants.climberMotor1CANID, superstructureCANBusName);
    m_ClimberMotor2 = new TalonFX(ClimberConstants.climberMotor2CANID, superstructureCANBusName);

    m_ClimberEncoder = new DutyCycleEncoder(climberEncoderPort, 1, 0.855);

    climberMMCruiseVelocity =
        new TunableNumber("Climber/Cruise Velocity", motionMagicCruiseVelocity);
    climberMMCruiseAcceleration =
        new TunableNumber("Climber/Cruise Acceleration", motionMagicCruiseAcceleration);
    climberMMKv = new TunableNumber("Climber/MM kV", motionMagickV);
    climberMMKa = new TunableNumber("Climber/MM kA", motionMagickA);

    m_PositionRequest = new MotionMagicExpoTorqueCurrentFOC(0);
    m_TestRequest = new TorqueCurrentFOC(Amps.of(0));

    PIDConfig.withKA(kA)
        .withKS(kS)
        .withKV(kV)
        .withKG(kG)
        .withKP(kP)
        .withKD(kD)
        .withGravityType(GravityTypeValue.Arm_Cosine)
        .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseVelocitySign);

    climberKp = new TunableNumber("Climber/kP", kP);
    climberKd = new TunableNumber("Climber/kD", kD);
    climberKg = new TunableNumber("Climber/kG", kG);
    climberKs = new TunableNumber("Climber/kS", kS);
    climberKa = new TunableNumber("Climber/kA", kA);
    climberKv = new TunableNumber("Climber/kV", kV);

    m_ClimberMotor1.getConfigurator().apply(getClimberMotorConfig());
    m_ClimberMotor2.getConfigurator().apply(getClimberMotorConfig());

    zeroPoint = new TunableNumber("Climber/Encoder Offset", 0);
    Timer.delay(5);
    resetPosition();
  }

  public void setState(ClimberPositions state) {
    m_ClimberState = state;
  }

  public ClimberPositions getState() {
    return m_ClimberState;
  }

  public double absoluteEncoderToRotations(double x) {
    return 132.2772 * Math.sin(3.36922 * x);
  }

  // TODO: rewrite so it does not take in a command xbox controller
  public void setPositionRequestWithController(CommandXboxController controller) {
    double leftY = controller.getLeftY();
    double rightY = controller.getRightY();
    if (Math.abs(leftY) > 0.2) {
      runCurrent(-controller.getLeftY() * 75);
    } else if (rightY > 0.2) {
      resetPosition();
      setPositionRequest(climbExtendPosition);
    } else if (rightY < -0.2) {
      resetPosition();
      setPositionRequest(climbPosition);
    } else {
      runCurrent(0);
    }
  }

  public void setPositionRequest(Angle position) {
    m_ClimberMotor1.setControl(
        m_PositionRequest.withPosition(position).withLimitReverseMotion(getAbsolutePosition() < 0));
    m_ClimberMotor2.setControl(
        m_PositionRequest.withPosition(position).withLimitReverseMotion(getAbsolutePosition() < 0));
  }

  // @Logged(name = "Climber Absolute Encoder", importance = Importance.CRITICAL)
  public double getAbsolutePosition() {
    return m_ClimberEncoder.get();
  }

  // @Logged(name = "Climber Position", importance = Importance.CRITICAL)
  public double getPositionRotations() {
    return getPosition().in(Rotations);
  }

  public Angle getPosition() {
    Measure<AngleUnit> motor1Position =
        BaseStatusSignal.getLatencyCompensatedValue(
            m_ClimberMotor1.getPosition(), m_ClimberMotor1.getVelocity());
    Measure<AngleUnit> motor2Position =
        BaseStatusSignal.getLatencyCompensatedValue(
            m_ClimberMotor2.getPosition(), m_ClimberMotor2.getVelocity());

    return Rotations.of((motor1Position.in(Rotations) + motor2Position.in(Rotations)) / 2);
  }

  public boolean atPosition() {
    return Math.abs(getPosition().in(Rotations) - ClimberConstants.climbPosition.in(Rotations))
        < ClimberConstants.climbDeadband;
  }

  public boolean isAtPositionAbsolute(Angle position) {
    return Math.abs(position.in(Rotations) - m_ClimberEncoder.get()) < climbDeadbandAbsolute;
  }

  public void runCurrent(double current) {
    current = MathUtil.applyDeadband(current, 5);
    m_ClimberMotor1.setControl(m_TestRequest.withOutput(current));
    m_ClimberMotor2.setControl(m_TestRequest.withOutput(current));
  }

  public void resetPosition() {
    m_ClimberMotor1.setPosition(absoluteEncoderToRotations(getAbsolutePosition()));
    m_ClimberMotor2.setPosition(absoluteEncoderToRotations(getAbsolutePosition()));
  }

  @Override
  public void periodic() {
    if (climberKp.getNumber() != PIDConfig.kP
        || climberKd.getNumber() != PIDConfig.kD
        || climberKs.getNumber() != PIDConfig.kS
        || climberKg.getNumber() != PIDConfig.kG
        || climberKa.getNumber() != PIDConfig.kA
        || climberKv.getNumber() != PIDConfig.kV) {
      PIDConfig.kP = climberKp.getNumber();
      PIDConfig.kD = climberKd.getNumber();
      PIDConfig.kS = climberKs.getNumber();
      PIDConfig.kG = climberKg.getNumber();
      PIDConfig.kA = climberKa.getNumber();
      PIDConfig.kV = climberKv.getNumber();

      m_ClimberMotor1.getConfigurator().apply(PIDConfig);
      m_ClimberMotor2.getConfigurator().apply(PIDConfig);
    }
    if (climberMMCruiseVelocity.getNumber() != MMConfig.MotionMagicCruiseVelocity
        || climberMMCruiseAcceleration.getNumber() != MMConfig.MotionMagicAcceleration
        || climberMMKv.getNumber() != MMConfig.MotionMagicExpo_kV
        || climberMMKa.getNumber() != MMConfig.MotionMagicExpo_kA) {

      MMConfig.MotionMagicCruiseVelocity = climberMMCruiseVelocity.getNumber();
      MMConfig.MotionMagicAcceleration = climberMMCruiseAcceleration.getNumber();
      MMConfig.MotionMagicExpo_kV = climberMMKv.getNumber();
      MMConfig.MotionMagicExpo_kA = climberMMKa.getNumber();

      m_ClimberMotor1.getConfigurator().apply(MMConfig);
      m_ClimberMotor2.getConfigurator().apply(MMConfig);
    }

    // switch (m_ClimberState) {
    //   case DEFAULT -> {
    //     setPositionRequest(Rotations.of(0));
    //   }
    //   case CLIMB -> {
    //     if (getLimitSwitch()) {
    //       setPositionRequest(getPosition());
    //     } else {
    //       setPositionRequest(ClimberConstants.climbPosition);
    //     }
    //   }
    // }
  }
}
