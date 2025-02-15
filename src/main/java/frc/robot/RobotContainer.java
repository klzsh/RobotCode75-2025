// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Strategy;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.commands.Drivetrain.DriveToPose;
import frc.robot.commands.Drivetrain.ResetHeading;
import frc.robot.commands.Drivetrain.SnapHoldRotation;
import frc.robot.commands.Drivetrain.TeleopSwerve;
import frc.robot.commands.Drivetrain.XStance;
import frc.robot.commands.EndEffector.Coral.ScoreL1;
import frc.robot.commands.EndEffector.Coral.ScoreL2;
import frc.robot.commands.EndEffector.Coral.ScoreL3;
import frc.robot.commands.EndEffector.Coral.ScoreL4;
import frc.robot.commands.EndEffector.IntakeCoral;
import frc.robot.commands.EndEffector.ScoreCoral;
import frc.robot.subsystems.Drivetrain.Swerve;
import frc.robot.subsystems.EndEffector.AlgaeIntake;
import frc.robot.subsystems.EndEffector.CoralIntake;
import frc.robot.subsystems.EndEffector.CoralIntake.CoralStates;
import frc.robot.subsystems.EndEffector.Elevator;
import frc.robot.subsystems.EndEffector.Elevator.ElevatorPositions;
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
  @Logged(name = "CoralCam")
  private final AprilTagCamera CoralCam =
      new AprilTagCamera("Center_Cam", VisionConstants.CenterCamPose);

  @Logged(name = "CenterCam")
  private final AprilTagCamera CenterCam =
      new AprilTagCamera("Coral_Cam", VisionConstants.CoralCamPose);

  @Logged(name = "Swerve")
  private final Swerve m_Swerve = new Swerve(CoralCam, CenterCam);

  @Logged(name = "Elevator")
  private final Elevator m_Elevator = new Elevator();

  @Logged(name = "Coral Intake")
  private final CoralIntake m_CoralIntake = new CoralIntake();
  
  @Logged(name = "Algae Intake")
  private final AlgaeIntake m_AlgaeIntake = new AlgaeIntake();

  // private final CANdleWrapper m_Wrapper = new CANdleWrapper();

  // define auto factory for autos
  // private final AutoFactory factory =
  //     new AutoFactory(
  //         m_Swerve::getPose, m_Swerve::setPose, m_Swerve::followSwerveSample, true, m_Swerve);

  // define OI controls
  private final Joystick m_LeftStick = new Joystick(OIConstants.leftStickPort);
  private final Joystick m_RightStick = new Joystick(OIConstants.rightStickPort);
  private final CommandXboxController m_Controller =
      new CommandXboxController(OIConstants.controllerPort);

  // define driver buttons
  private final JoystickButton robotRelative =
      new JoystickButton(m_RightStick, OIConstants.robotRelativeButton);
  private final JoystickButton resetHeading =
      new JoystickButton(m_LeftStick, OIConstants.resetHeadingButton);
  private final JoystickButton Xstance = new JoystickButton(m_RightStick, OIConstants.xstance);

  private final JoystickButton alignButton = new JoystickButton(m_LeftStick, 2);
  private final JoystickButton testDrivePose = new JoystickButton(m_RightStick, 5);

  private final JoystickButton holdButton =
      new JoystickButton(m_RightStick, OIConstants.holdHeadingButton);

  private final SendableChooser<Command> m_AutoChooser = new SendableChooser<>();

  //   private final Map<Integer, Command> m_AutoMap = Map.of(
  //     1, new ScoreL1(m_Elevator, m_CoralIntake),
  //     3, new ScoreL4(m_Elevator, m_CoralIntake), // TODO add left/right distinction
  //     4, new ScoreL4(m_Elevator, m_CoralIntake),
  //     6, new IntakeCoral(m_CoralIntake) //TODO add left/middle/right distinction
  //   );
  //   private final AutoSelector m_Selector =
  //       new AutoSelector(
  //           m_AutoMap,
  //           m_Swerve,
  //           new ArrayList<Command>(),
  //           new ArrayList<Command>());

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    DriverStation.silenceJoystickConnectionWarning(true);
    configureDefaultCommands();
    configureBindings();
    // configureChooser();
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
            () -> -m_RightStick.getX(),
            false,
            !robotRelative.getAsBoolean()));
    // m_Wrapper.setDefaultCommand(new LEDsDefaultCommand(m_Wrapper));
    m_Elevator.setDefaultCommand(
        new InstantCommand(() -> m_Elevator.setPosition(ElevatorPositions.HOME, false), m_Elevator)
            .repeatedly());
    m_CoralIntake.setDefaultCommand(
        new InstantCommand(() -> m_CoralIntake.setState(CoralStates.DEFAULT), m_CoralIntake)
            .repeatedly());

    m_AlgaeInt
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

    m_Controller.a().whileTrue(new ScoreL1(m_Elevator, m_CoralIntake));
    m_Controller.x().whileTrue(new ScoreL2(m_Elevator, m_CoralIntake));
    m_Controller.y().whileTrue(new ScoreL3(m_Elevator, m_CoralIntake));
    m_Controller.b().whileTrue(new ScoreL4(m_Elevator, m_CoralIntake));

    m_Controller.povUp().whileTrue(new IntakeCoral(m_CoralIntake));
    m_Controller.povDown().whileTrue(new ScoreCoral(m_CoralIntake));

    testDrivePose.whileTrue(
        new DriveToPose(m_Swerve, new Pose2d(2.9, 4.0, new Rotation2d(0)), false));
    // m_Controller
    //     .a()
    //     .whileTrue(
    //         new InstantCommand(
    //                 () -> m_Elevator.setPosition(ElevatorPositions.L1, false), m_Elevator)
    //             .repeatedly());
    // m_Controller
    //     .b()
    //     .whileTrue(
    //         new InstantCommand(
    //                 () -> m_Elevator.setPosition(ElevatorPositions.L2, false), m_Elevator)
    //             .repeatedly());
    // m_Controller
    //     .x()
    //     .whileTrue(
    //         new InstantCommand(
    //                 () -> m_Elevator.setPosition(ElevatorPositions.L3, false), m_Elevator)
    //             .repeatedly());
    // m_Controller
    //     .y()
    //     .whileTrue(
    //         new InstantCommand(
    //                 () -> m_Elevator.setPosition(ElevatorPositions.L4, false), m_Elevator)
    //             .repeatedly());
  }

  private void configureChooser() {}

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // return m_AutoChooser.getSelected();
    return null;
  }
}
