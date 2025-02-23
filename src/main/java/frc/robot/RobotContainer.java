// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Inches;
import static frc.robot.Constants.ClimberConstants.*;
import static frc.robot.Constants.OIConstants.*;
import static frc.robot.Constants.VisionConstants.*;

import choreo.auto.AutoFactory;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Strategy;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.util.FieldPose;
import frc.lib.util.FieldPose.FieldElement;
import frc.lib.util.FieldPose.Offset;
import frc.robot.commands.Autonomous.TestAuto;
import frc.robot.commands.Drivetrain.DriveToPose;
import frc.robot.commands.Drivetrain.DriveVisionAlign;
import frc.robot.commands.Drivetrain.ResetHeading;
import frc.robot.commands.Drivetrain.SnapHoldRotation;
import frc.robot.commands.Drivetrain.TeleopSwerve;
import frc.robot.commands.Drivetrain.XStance;
import frc.robot.commands.EndEffector.Algae.DeAlgaefy;
import frc.robot.commands.EndEffector.Coral.IntakeCoral;
import frc.robot.commands.EndEffector.Coral.ScoreCoral;
import frc.robot.commands.EndEffector.Coral.ScoreL1;
import frc.robot.commands.EndEffector.Coral.ScoreL2;
import frc.robot.commands.EndEffector.Coral.ScoreL3;
import frc.robot.commands.EndEffector.Coral.ScoreL4;
import frc.robot.subsystems.Drivetrain.PoseAlignController;
import frc.robot.subsystems.Drivetrain.Swerve;
import frc.robot.subsystems.Drivetrain.VisionTranslationController;
import frc.robot.subsystems.EndEffector.AlgaeIntake;
import frc.robot.subsystems.EndEffector.AlgaeIntake.AlgaeStates;
import frc.robot.subsystems.EndEffector.AlgaePivot;
import frc.robot.subsystems.EndEffector.AlgaePivot.PivotState;
import frc.robot.subsystems.EndEffector.CoralIntake;
import frc.robot.subsystems.EndEffector.CoralIntake.CoralStates;
import frc.robot.subsystems.EndEffector.Elevator;
import frc.robot.subsystems.EndEffector.Elevator.ElevatorPositions;
import frc.robot.subsystems.EndGame.Climber;
import frc.robot.subsystems.Util.CANRangeWrapper;
import frc.robot.subsystems.Vision.AprilTagCamera;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
@Logged(strategy = Strategy.OPT_IN)
public class RobotContainer {
  // define subsystems first
  @Logged(name = "CenterCam")
  private final AprilTagCamera CenterCam = new AprilTagCamera("Center_Cam", CenterCamPose);

  @Logged(name = "CoralCam")
  private final AprilTagCamera CoralCam = new AprilTagCamera("Coral_Cam", CoralCamPose);

  @Logged(name = "Swerve")
  private final Swerve m_Swerve = new Swerve(CoralCam, CenterCam);

  //   @Logged(name = "Elevator")
  private final Elevator m_Elevator = new Elevator();

  //   @Logged(name = "Coral Intake")
  private final CoralIntake m_CoralIntake = new CoralIntake();

  @Logged(name = "Climber")
  private final Climber m_Climber = new Climber();

  //   @Logged(name = "Algae Intake")
  private final AlgaeIntake m_AlgaeIntake = new AlgaeIntake();

  //   @Logged(name = "Algae Pivot")
  private final AlgaePivot m_AlgaePivot = new AlgaePivot();

  private final CANRangeWrapper m_CANRange = new CANRangeWrapper(Inches.of(37));

  //   @Logged(name = "Algae Lidar Sensor")
  //   private final LidarDistance distanceSensor = new LidarDistance(Inches.of(36));

  // private final CANdleWrapper m_Wrapper = new CANdleWrapper();

  // define drivetrain controllers
  private final PoseAlignController m_PoseAlignController = new PoseAlignController(m_Swerve);
  private final VisionTranslationController m_VisionController =
      new VisionTranslationController(m_Swerve, CoralCam, CenterCam);

  // define OI controls
  private final Joystick m_LeftStick = new Joystick(leftStickPort);
  private final Joystick m_RightStick = new Joystick(rightStickPort);
  private final CommandXboxController m_Controller = new CommandXboxController(controllerPort);

  // define driver buttons
  private final JoystickButton robotRelative =
      new JoystickButton(m_RightStick, robotRelativeButton);
  private final JoystickButton resetHeading = new JoystickButton(m_LeftStick, resetHeadingButton);
  private final JoystickButton Xstance = new JoystickButton(m_RightStick, xstance);

  private final JoystickButton AlignLeft = new JoystickButton(m_LeftStick, 1);
  private final JoystickButton AlignRight = new JoystickButton(m_RightStick, 1);

  private final JoystickButton holdButton = new JoystickButton(m_RightStick, holdHeadingButton);

  private final SendableChooser<Command> m_AutoChooser = new SendableChooser<>();

  private final AutoFactory m_Factory =
      new AutoFactory(
          m_Swerve::getPose, m_Swerve::setPose, m_Swerve::followSwerveSample, true, m_Swerve);

  //   private final Map<Integer, Command> m_AutoMap =
  //       Map.of(
  //           1, new ScoreL1(m_Elevator, m_CoralIntake),
  //           3, new ScoreL4(m_Elevator, m_CoralIntake), // TODO add left/right distinction
  //           4, new ScoreL4(m_Elevator, m_CoralIntake),
  //           6, new IntakeCoral(m_CoralIntake) // TODO add left/middle/right distinction
  //           );
  //   private final AutoSelector m_Selector =
  //       new AutoSelector(m_AutoMap, m_Swerve, new ArrayList<Command>(), new
  // ArrayList<Command>());

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    DriverStation.silenceJoystickConnectionWarning(true);
    configureDefaultCommands();
    configureBindings();
    configureChooser();
  }

  private void configureDefaultCommands() {
    m_Swerve.setDefaultCommand(
        new TeleopSwerve(
            m_Swerve,
            // for some reason, the makers of WPILIB decided that joystick coordinates and robot
            // coordinates should be flipped. I don't know what drove them to do that, but due to
            // this decision, we have to negate the stick values.
            () -> -m_LeftStick.getY(),
            () -> -m_LeftStick.getX(),
            () -> -m_RightStick.getX()));
    // m_Wrapper.setDefaultCommand(new LEDsDefaultCommand(m_Wrapper));
    m_Elevator.setDefaultCommand(
        new InstantCommand(() -> m_Elevator.setPosition(ElevatorPositions.HOME, false), m_Elevator)
            .repeatedly());
    m_CoralIntake.setDefaultCommand(
        new InstantCommand(() -> m_CoralIntake.setState(CoralStates.DEFAULT), m_CoralIntake)
            .repeatedly());
    m_AlgaeIntake.setDefaultCommand(
        new InstantCommand(() -> m_AlgaeIntake.resetAlgaeState(), m_AlgaeIntake).repeatedly());
    m_AlgaePivot.setDefaultCommand(
        new InstantCommand(() -> m_AlgaePivot.resetPivotState(), m_AlgaePivot).repeatedly());
    // m_Climber.setDefaultCommand(
    //     new InstantCommand(() -> m_Climber.setState(ClimberPositions.DEFAULT), m_Climber)
    //         .repeatedly());
    m_Climber.setDefaultCommand(
        new InstantCommand(() -> m_Climber.setPositionRequestWithController(m_Controller), m_Climber));

    // m_Climber.setDefaultCommand(
    //     new InstantCommand(() -> {
    //         if (m_Controller.getLeftY() < -0.1) {
    //             m_Climber.setPositionRequest(climbPositionAbsoluteStart);
    //         } else if (m_Controller.getLeftY() > 0.1) {
    //             m_Climber.setPositionRequest(climbPositionAbsoluteFinish);
    //         }
    //     }, m_Climber)
    // );
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    resetHeading.onTrue(new ResetHeading(m_Swerve));
    Xstance.whileTrue(new XStance(m_Swerve));
    holdButton.whileTrue(
        new SnapHoldRotation(m_Swerve, () -> -m_LeftStick.getY(), () -> -m_LeftStick.getX()));
    // Score L* commands
    m_Controller
        .a()
        .and(() -> m_Controller.getRightTriggerAxis() <= 0.15)
        .whileTrue(new ScoreL1(m_Elevator, m_CoralIntake));
    m_Controller
        .x()
        .and(() -> m_Controller.getLeftTriggerAxis() <= 0.15)
        .and(() -> m_Controller.getRightTriggerAxis() <= 0.15)
        .whileTrue(new ScoreL2(m_Elevator, m_CoralIntake));
    m_Controller
        .y()
        .and(() -> m_Controller.getLeftTriggerAxis() <= 0.15)
        .and(() -> m_Controller.getRightTriggerAxis() <= 0.15)
        .whileTrue(new ScoreL3(m_Elevator, m_CoralIntake));
    m_Controller
        .b()
        .and(() -> m_Controller.getRightTriggerAxis() <= 0.15)
        .whileTrue(new ScoreL4(m_Elevator, m_CoralIntake));
    // algae ground pickup & scoring
    m_Controller
        .povRight()
        .whileTrue(
            new InstantCommand(
                    () -> m_AlgaeIntake.setAlgaeState(AlgaeStates.OUTAKING), m_AlgaeIntake)
                .repeatedly());
    m_Controller
        .leftBumper()
        .whileTrue(
            new InstantCommand(
                    () -> {
                      m_AlgaeIntake.setAlgaeState(AlgaeStates.INTAKING);
                      m_AlgaePivot.setPivotState(PivotState.DEALGAEFY);
                    },
                    m_AlgaeIntake,
                    m_AlgaePivot)
                .repeatedly()
                .until(() -> m_AlgaeIntake.getAlgaeState() == AlgaeStates.HASGAMEPIECE));

    m_Controller.povUp().whileTrue(new IntakeCoral(m_CoralIntake));
    m_Controller.povDown().whileTrue(new ScoreCoral(m_CoralIntake));

    AlignLeft.and(() -> !AlignRight.getAsBoolean())
        .whileTrue(
            new DriveVisionAlign(
                m_Swerve,
                new FieldPose(Alliance.Blue, FieldElement.RL, Offset.LEFT),
                m_PoseAlignController,
                m_VisionController));
    AlignRight.and(() -> !AlignLeft.getAsBoolean())
        .whileTrue(
            new DriveToPose(
                m_Swerve,
                m_PoseAlignController,
                new FieldPose(Alliance.Blue, FieldElement.RL, Offset.RIGHT),
                false)
            // new DriveVisionAlign(
            //     m_Swerve,
            //     new FieldPose(Alliance.Blue, FieldElement.RL, Offset.RIGHT),
            //     m_PoseAlignController,
            //     m_VisionController));
            );
    AlignRight.and(() -> AlignLeft.getAsBoolean())
        .whileTrue(
            new DriveToPose(
                m_Swerve,
                m_PoseAlignController,
                new FieldPose(Alliance.Blue, FieldElement.RL, Offset.MID),
                false));

    // m_Controller
    //     .rightBumper()
    //     .whileTrue(
    //         new DriveVisionAlign(
    //             m_Swerve,
    //             new FieldPose(Alliance.Blue, FieldElement.RL, Offset.LEFT),
    //             m_PoseAlignController,
    //             m_VisionController));

    // manual elevator overrides

    m_Controller
        .a()
        .and(() -> m_Controller.getRightTriggerAxis() >= 0.15)
        .whileTrue(
            new InstantCommand(
                    () -> m_Elevator.setPosition(ElevatorPositions.L1, false), m_Elevator)
                .repeatedly());
    m_Controller
        .x()
        .and(() -> m_Controller.getRightTriggerAxis() >= 0.15)
        .and(() -> m_Controller.getLeftTriggerAxis() <= 0.15)
        .whileTrue(
            new InstantCommand(
                    () -> m_Elevator.setPosition(ElevatorPositions.L2, false), m_Elevator)
                .repeatedly());
    m_Controller
        .y()
        .and(() -> m_Controller.getRightTriggerAxis() >= 0.15)
        .and(() -> m_Controller.getLeftTriggerAxis() <= 0.15)
        .whileTrue(
            new InstantCommand(
                    () -> m_Elevator.setPosition(ElevatorPositions.L3, false), m_Elevator)
                .repeatedly());
    m_Controller
        .b()
        .and(() -> m_Controller.getRightTriggerAxis() >= 0.15)
        .whileTrue(
            new InstantCommand(
                    () -> m_Elevator.setPosition(ElevatorPositions.L4, false), m_Elevator)
                .repeatedly());
    // dealgaefy commands
    m_Controller
        .x()
        .and(() -> m_Controller.getLeftTriggerAxis() > 0.15)
        .and(() -> m_Controller.getRightTriggerAxis() <= 0.15)
        .whileTrue(new DeAlgaefy(m_Elevator, m_AlgaeIntake, m_AlgaePivot, true));
    m_Controller
        .y()
        .and(() -> m_Controller.getLeftTriggerAxis() > 0.15)
        .and(() -> m_Controller.getRightTriggerAxis() <= 0.15)
        .whileTrue(new DeAlgaefy(m_Elevator, m_AlgaeIntake, m_AlgaePivot, false));
    robotRelative
        .onTrue(new InstantCommand(() -> m_Swerve.toggleRobotRelative()))
        .onFalse(new InstantCommand(() -> m_Swerve.toggleFieldRelative()));
  }

  private void configureChooser() {
    // m_Selector.setupAutoTab();
    // m_Selector.clearField();
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // return m_AutoChooser.getSelected();
    return new TestAuto(m_Factory, m_CoralIntake, m_Swerve, m_Elevator, m_VisionController);
    // return null;
  }
}
