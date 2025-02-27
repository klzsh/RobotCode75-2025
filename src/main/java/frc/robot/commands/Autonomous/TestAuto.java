// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autonomous;

import choreo.auto.AutoFactory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.lib.util.FieldPose;
import frc.lib.util.FieldPose.FieldElement;
import frc.lib.util.FieldPose.Offset;
import frc.robot.Constants.AutoConstants;
import frc.robot.commands.Drivetrain.DriveToPose;
import frc.robot.commands.Drivetrain.RotateToSimilarFace;
import frc.robot.commands.Drivetrain.TranslateToBranch;
import frc.robot.commands.Drivetrain.VisionAlign;
import frc.robot.commands.EndEffector.Coral.IntakeCoral;
import frc.robot.subsystems.Drivetrain.PoseAlignController;
import frc.robot.subsystems.Drivetrain.Swerve;
import frc.robot.subsystems.Drivetrain.VisionTranslationController;
import frc.robot.subsystems.EndEffector.CoralIntake;
import frc.robot.subsystems.EndEffector.Elevator;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class TestAuto extends SequentialCommandGroup {
  /** Creates a new TestAuto. */
  private final AutoFactory m_Factory;

  public TestAuto(
      AutoFactory factory,
      CoralIntake CoralIntake,
      Swerve swerve,
      Elevator elevator,
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
      Commands.runOnce(() -> {
      swerve.zeroGyro(Rotation2d.fromDegrees(180));
      swerve.setPose(
        new Pose2d(AutoConstants.blueStartPositions.get("st").getX(), AutoConstants.blueStartPositions.get("st").getY(), Rotation2d.fromDegrees(180)));
      }),
      m_Factory.trajectoryCmd("st-rtl")//,
      // new DriveToPose(
      //           swerve,
      //           poseAlignController,
      //           new FieldPose(Alliance.Blue, FieldElement.RTL, Offset.LEFT),
      //           false));
      // new RotateToSimilarFace(swerve),
      // new TranslateToBranch(swerve, visionController.m_RightFacingCamera, true, poseAlignController, new FieldPose(Alliance.Blue, FieldElement.RTL, Offset.LEFT))
      );
  }
}
