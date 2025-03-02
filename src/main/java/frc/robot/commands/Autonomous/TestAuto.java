// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autonomous;

import choreo.auto.AutoFactory;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.Drivetrain.RotateToSimilarFace;
import frc.robot.commands.EndEffector.Coral.ScoreL4;
import frc.robot.subsystems.Drivetrain.PoseAlignController;
import frc.robot.subsystems.Drivetrain.Swerve;
import frc.robot.subsystems.Drivetrain.VisionTranslationController;
import frc.robot.subsystems.EndEffector.CoralIntake;
import frc.robot.subsystems.EndEffector.CoralIntake.CoralStates;
import frc.robot.subsystems.EndEffector.Elevator;
import frc.robot.subsystems.Vision.AprilTagCamera;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class TestAuto extends SequentialCommandGroup {
  /** Creates a new TestAuto. */
  private final AutoFactory m_Factory;

  public TestAuto(
      AutoFactory factory,
      CoralIntake coralIntake,
      Swerve swerve,
      Elevator elevator,
      AprilTagCamera leftCamera,
      AprilTagCamera rightCamera,
      VisionTranslationController visionController,
      PoseAlignController poseAlignController) {
    m_Factory = factory;
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    // Went way faster than Choreo velocity constraint
    // addCommands(
    //     new SequentialCommandGroup(
    //         m_Factory.trajectoryCmd("rl-ht"),
    //         new IntakeCoral(CoralIntake),
    //         m_Factory.trajectoryCmd("ht-rl"),
    //         new VisionAlign(
    //             swerve,
    //             new FieldPose(Alliance.Blue, FieldElement.RL, Offset.LEFT),
    //             visionController)
    //         // new ScoreL4(elevator, CoralIntake)
    //         ));
    addCommands(
        // Commands.runOnce(
        //     () -> {
        //       swerve.zeroGyro(Rotation2d.fromDegrees(180));
        //     }),
        // m_Factory.resetOdometry("st-rtl"),
        // m_Factory.trajectoryCmd("st-rtl"),
        // new InstantCommand(() -> swerve.stopModules(), swerve),
        // // new RotateToSimilarFace(swerve),
        // // new AlignToBranch(swerve, rightCamera, true, poseAlignController, new
        // // FieldPose(Alliance.Blue, FieldElement.RTL, Offset.LEFT)),
        // new ScoreL4(elevator, coralIntake),
        // m_Factory.trajectoryCmd("rtl-ht"),
        // // new SnapHoldRotation(swerve, Rotation2d.fromDegrees(36), () -> 0, () -> 0),
        // new InstantCommand(() -> coralIntake.setState(CoralStates.INTAKING)),
        // new WaitCommand(0.5),
        // m_Factory.trajectoryCmd("ht-rl"),
        // new RotateToSimilarFace(swerve),
        // new ScoreL4(elevator, coralIntake)

        Commands.runOnce(
          () -> {
            swerve.zeroGyro(Rotation2d.fromDegrees(180));
          }),
      m_Factory.resetOdometry("sm-dl"),
      m_Factory.trajectoryCmd("sm-dl"),
      new InstantCommand(() -> swerve.stopModules(), swerve),
      // new RotateToSimilarFace(swerve),
      // new AlignToBranch(swerve, rightCamera, true, poseAlignController, new
      // FieldPose(Alliance.Blue, FieldElement.RTL, Offset.LEFT)),
      new ScoreL4(elevator, coralIntake),
      m_Factory.trajectoryCmd("d-hb"),
      // new SnapHoldRotation(swerve, Rotation2d.fromDegrees(36), () -> 0, () -> 0),
      new InstantCommand(() -> coralIntake.setState(CoralStates.INTAKING)),
      new WaitCommand(0.5),
      m_Factory.trajectoryCmd("hb-fl"),
      new RotateToSimilarFace(swerve),
      new ScoreL4(elevator, coralIntake)

        
        // m_Factory.trajectoryCmd("rtl-ht"),
        // new InstantCommand(() -> swerve.stopModules(), swerve),
        // new DriveToPose(
        //     swerve,
        //     poseAlignController,
        //     new FieldPose(Alliance.Blue, FieldElement.HT, Offset.MID),
        //     false),
        // new WaitCommand(0.5),
        // m_Factory.trajectoryCmd("ht-rtr"),
        // new InstantCommand(() -> swerve.stopModules(), swerve),
        // new DriveToPose(
        //     swerve,
        //     poseAlignController,
        //     new FieldPose(Alliance.Blue, FieldElement.RL, Offset.LEFT),
        //     false)
        // new RotateToSimilarFace(swerve),
        // new TranslateToBranch(swerve, visionController.m_RightFacingCamera, true,
        // poseAlignController, new FieldPose(Alliance.Blue, FieldElement.RTL, Offset.LEFT))
        );
  }
}
