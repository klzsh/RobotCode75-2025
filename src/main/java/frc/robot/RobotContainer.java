// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.Constants.ClimberConstants.climbExtendPosition;
import static frc.robot.Constants.ClimberConstants.climbPosition;
import static frc.robot.Constants.OIConstants.*;
import static frc.robot.Constants.VisionConstants.*;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Importance;
import edu.wpi.first.epilogue.Logged.Strategy;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.dashboard.ActionFactory;
import frc.lib.dashboard.AutoSelector;
import frc.lib.util.FieldPose.Offset;
import frc.robot.commands.Drivetrain.AlignToCoralStation;
import frc.robot.commands.Drivetrain.AlignToReef;
import frc.robot.commands.Drivetrain.ResetHeading;
import frc.robot.commands.Drivetrain.RotateToSimilarCoralStation;
import frc.robot.commands.Drivetrain.RotateToSimilarFace;
import frc.robot.commands.Drivetrain.TeleopSwerve;
import frc.robot.commands.Drivetrain.XStance;
import frc.robot.commands.EndEffector.Algae.DeAlgaefy;
import frc.robot.commands.EndEffector.Coral.IntakeCoral;
import frc.robot.commands.EndEffector.Coral.ScoreCoral;
import frc.robot.subsystems.Drivetrain.ChezyController;
import frc.robot.subsystems.Drivetrain.RotationController;
import frc.robot.subsystems.Drivetrain.Swerve;
import frc.robot.subsystems.EndEffector.AlgaeIntake;
import frc.robot.subsystems.EndEffector.AlgaeIntake.AlgaeStates;
import frc.robot.subsystems.EndEffector.AlgaePivot;
import frc.robot.subsystems.EndEffector.AlgaePivot.PivotState;
import frc.robot.subsystems.EndEffector.CoralIntake;
import frc.robot.subsystems.EndEffector.CoralIntake.CoralStates;
import frc.robot.subsystems.EndEffector.Elevator;
import frc.robot.subsystems.EndEffector.Elevator.ElevatorPositions;
import frc.robot.subsystems.EndGame.Climber;
import frc.robot.subsystems.Vision.AprilTagCamera;
import frc.robot.subsystems.Vision.ObjectDetetectorCamera;
import frc.robot.subsystems.Vision.OdometryWrapper;
import frc.robot.subsystems.Vision.YoloController;
import java.util.ArrayList;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
@Logged(strategy = Strategy.OPT_IN)
public class RobotContainer {
  // define subsystems first
  //   @Logged(name = "Left Facing Mod Cam", importance = Importance.CRITICAL)
  private final AprilTagCamera m_LeftFacingCamera =
      new AprilTagCamera("Center_Cam", LeftFacingCameraPose);

  //   @Logged(name = "Right Facing Mod Cam", importance = Importance.CRITICAL)
  private final AprilTagCamera m_RightFacingCamera =
      new AprilTagCamera("Coral_Cam", RightFacingCameraPose);

  //   @Logged(name = "HP Cam", importance = Importance.CRITICAL)
  private final AprilTagCamera m_HPCamera = new AprilTagCamera("HP_Cam", HPCameraPose);

  //   private final ObjectDetetectorCamera m_CageDetetectorCamera =
  //       new ObjectDetetectorCamera("Cage_Cam");

  // @Logged(name = "Branch Cam")
  private final ObjectDetetectorCamera m_BranchCamera = new ObjectDetetectorCamera("Branch_Cam");

  @Logged(name = "Swerve", importance = Importance.CRITICAL)
  private final Swerve m_Swerve = new Swerve(m_LeftFacingCamera, m_RightFacingCamera, m_HPCamera);

  @Logged(name = "Elevator", importance = Importance.CRITICAL)
  private final Elevator m_Elevator = new Elevator();

  @Logged(name = "Coral Intake", importance = Importance.CRITICAL)
  private final CoralIntake m_CoralIntake = new CoralIntake();

  @Logged(name = "Climber", importance = Importance.CRITICAL)
  private final Climber m_Climber = new Climber();

  // @Logged(name = "Algae Intake", importance = Importance.CRITICAL)
  private final AlgaeIntake m_AlgaeIntake = new AlgaeIntake();

  // @Logged(name = "Algae Pivot", importance = Importance.CRITICAL)
  private final AlgaePivot m_AlgaePivot = new AlgaePivot();

  // define drivetrain controllers
  @Logged private final ChezyController m_ChezyController = new ChezyController(m_Swerve);

  @Logged
  private final YoloController m_YoloController = new YoloController(m_Swerve, m_BranchCamera);

  @Logged private final RotationController m_RotationController = new RotationController(m_Swerve);

  // odometry
  private final OdometryWrapper m_Odometry =
      new OdometryWrapper(m_Swerve, m_LeftFacingCamera, m_RightFacingCamera, m_HPCamera);

  // define OI controls
  private final Joystick m_LeftStick = new Joystick(leftStickPort);
  private final Joystick m_RightStick = new Joystick(rightStickPort);
  private final CommandXboxController m_Controller = new CommandXboxController(controllerPort);

  // define driver buttons
  private final JoystickButton robotRelative =
      new JoystickButton(m_RightStick, robotRelativeButton); // center button
  private final JoystickButton resetHeading =
      new JoystickButton(m_LeftStick, resetHeadingButton); // left button
  private final JoystickButton Xstance = new JoystickButton(m_RightStick, xstanceButton);

  private final JoystickButton AlignLeft =
      new JoystickButton(m_LeftStick, autoAlignButton); // trigger
  private final JoystickButton AlignRight =
      new JoystickButton(m_RightStick, autoAlignButton); // trigger

  private final JoystickButton SimilarFaceRotate =
      new JoystickButton(m_RightStick, rotateToSimilarFaceButton); // left button

  private final JoystickButton AlgaeAlign =
      new JoystickButton(m_LeftStick, algaeAlignButton); // center button

  private final JoystickButton resetBranchCam =
      new JoystickButton(m_LeftStick, resetBranchCamButton);

  private final JoystickButton HPRotate = new JoystickButton(m_RightStick, hpRotateButton);
  private final JoystickButton HPAlign = new JoystickButton(m_LeftStick, hpRotateButton);

  //   private final JoystickButton holdButton =
  //       new JoystickButton(m_RightStick, holdHeadingButton); // center button, ts is used twice?

  private final ActionFactory m_ActionFactory =
      new ActionFactory(
          m_Swerve,
          m_Elevator,
          m_CoralIntake,
          m_AlgaePivot,
          m_AlgaeIntake,
          m_ChezyController,
          m_YoloController);

  private final AutoSelector m_AutoSelector =
      new AutoSelector(
          m_ActionFactory, m_Swerve, new ArrayList<Command>(), new ArrayList<Command>());

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    DriverStation.silenceJoystickConnectionWarning(true);
    configureDefaultCommands();
    configureJoystickBinds();
    configureControllerBinds();
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

    m_Elevator.setDefaultCommand(
        new InstantCommand(() -> m_Elevator.setPosition(ElevatorPositions.HOME, false), m_Elevator)
            .repeatedly());
    m_CoralIntake.setDefaultCommand(
        new InstantCommand(() -> m_CoralIntake.setState(m_CoralIntake.getState()), m_CoralIntake)
            .repeatedly());
    m_AlgaeIntake.setDefaultCommand(
        new InstantCommand(() -> m_AlgaeIntake.resetAlgaeState(), m_AlgaeIntake).repeatedly());
    m_AlgaePivot.setDefaultCommand(
        new InstantCommand(() -> m_AlgaePivot.resetPivotState(), m_AlgaePivot).repeatedly());
    m_Climber.setDefaultCommand(new InstantCommand(() -> m_Climber.runCurrent(0), m_Climber));
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
  private void configureJoystickBinds() {
    resetHeading.onTrue(new ResetHeading(m_Swerve));
    Xstance.whileTrue(new XStance(m_Swerve));
    // debug
    AlignLeft.whileTrue(
        new AlignToReef(
            m_Swerve, m_ChezyController, m_YoloController, m_BranchCamera, Offset.LEFT));
    AlignRight.whileTrue(
        new AlignToReef(
            m_Swerve, m_ChezyController, m_YoloController, m_BranchCamera, Offset.RIGHT));
    // AlignLeft.and(() -> !m_CoralIntake.getBeamBreak())
    //     .whileTrue(new AlignToCoralStation(m_Swerve, m_ChezyController, Offset.LEFT));
    // // AlignLeft.whileTrue(new YoloBranchAlign(m_Swerve, m_YoloController, true));
    // AlignRight.and(() -> !m_CoralIntake.getBeamBreak())
    //     .whileTrue(new AlignToCoralStation(m_Swerve, m_ChezyController, Offset.RIGHT));
    // debug command
    AlgaeAlign.whileTrue(
        new AlignToReef(m_Swerve, m_ChezyController, m_YoloController, m_BranchCamera, Offset.MID));

    SimilarFaceRotate.whileTrue(new RotateToSimilarFace(m_Swerve, m_RotationController));
    HPRotate.whileTrue(
        new RotateToSimilarCoralStation(
            m_Swerve, () -> -m_LeftStick.getY(), () -> -m_LeftStick.getX()));
    HPAlign.whileTrue(new AlignToCoralStation(m_Swerve, m_ChezyController, Offset.MID));
    robotRelative
        .onTrue(new InstantCommand(() -> m_Swerve.toggleRobotRelative()))
        .onFalse(new InstantCommand(() -> m_Swerve.toggleFieldRelative()));

    resetBranchCam.onTrue(
        new InstantCommand(() -> m_BranchCamera.reloadPipeline()).ignoringDisable(true));
  }

  public void configureControllerBinds() {

    // m_Controller.povRight().whileTrue(new YoloBranchAlign(m_Swerve, m_YoloController, true));

    // m_Controller.rightBumper().whileTrue(new OdometryToReef(m_Swerve, m_ChezyController,
    // Offset.LEFT));

    // algae commands
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
                      m_AlgaePivot.setPivotState(PivotState.GROUNDINTAKE);
                    },
                    m_AlgaeIntake,
                    m_AlgaePivot)
                .repeatedly()
                .until(() -> m_AlgaeIntake.getAlgaeState() == AlgaeStates.HASGAMEPIECE));
    m_Controller
        .rightBumper()
        .whileTrue(
            new InstantCommand(
                    () -> m_AlgaeIntake.setAlgaeState(AlgaeStates.INTAKING), m_AlgaeIntake)
                .repeatedly()
                .until(() -> m_AlgaeIntake.getAlgaeState() == AlgaeStates.HASGAMEPIECE));
    // coral commands
    m_Controller.povUp().onTrue(new IntakeCoral(m_CoralIntake));
    m_Controller.povDown().whileTrue(new ScoreCoral(m_CoralIntake, m_Elevator));
    m_Controller
        .povLeft()
        .whileTrue(
            new InstantCommand(() -> m_CoralIntake.setState(CoralStates.REVERSING), m_CoralIntake)
                .repeatedly());
    // elev. positions
    m_Controller
        .a()
        .whileTrue(
            new InstantCommand(
                    () -> m_Elevator.setPosition(ElevatorPositions.L1, false), m_Elevator)
                .repeatedly());
    m_Controller
        .x()
        .whileTrue(
            new InstantCommand(
                    () -> m_Elevator.setPosition(ElevatorPositions.L2, false), m_Elevator)
                .repeatedly());
    m_Controller
        .y()
        .whileTrue(
            new InstantCommand(
                    () -> m_Elevator.setPosition(ElevatorPositions.L3, false), m_Elevator)
                .repeatedly());
    m_Controller
        .b()
        .whileTrue(
            new InstantCommand(
                    () -> m_Elevator.setPosition(ElevatorPositions.L4, false), m_Elevator)
                .repeatedly());

    m_Controller.rightTrigger(0.15).whileTrue(new ScoreCoral(m_CoralIntake, m_Elevator));
    // dealgaefy commands
    m_Controller
        .x()
        .and(() -> m_Controller.getLeftTriggerAxis() > 0.15)
        .whileTrue(new DeAlgaefy(m_Elevator, m_AlgaeIntake, m_AlgaePivot, true));
    m_Controller
        .y()
        .and(() -> m_Controller.getLeftTriggerAxis() > 0.15)
        .whileTrue(new DeAlgaefy(m_Elevator, m_AlgaeIntake, m_AlgaePivot, false));
    // climber commands
    // Left Y, more than 0.15
    m_Controller
        .axisLessThan(1, -0.15)
        .whileTrue(
            new InstantCommand(() -> m_Climber.runCurrent(-m_Controller.getLeftY() * 75), m_Climber)
                .repeatedly());
    m_Controller
        .axisGreaterThan(1, 0.15)
        .whileTrue(
            new InstantCommand(() -> m_Climber.runCurrent(-m_Controller.getLeftY() * 75), m_Climber)
                .repeatedly());

    m_Controller
        .axisLessThan(5, -0.15)
        .whileTrue(
            new InstantCommand(() -> m_Climber.setPositionRequest(climbPosition), m_Climber)
                .repeatedly());
    m_Controller
        .axisGreaterThan(5, 0.15)
        .whileTrue(
            new InstantCommand(() -> m_Climber.setPositionRequest(climbExtendPosition), m_Climber)
                .repeatedly());
  }

  private void configureChooser() {
    m_AutoSelector.setupAutoTab();
    m_AutoSelector.clearAll();
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    m_AutoSelector.generatePaths();
    return m_AutoSelector.getAutoCommand();
  }
}
