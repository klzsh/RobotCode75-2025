// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autonomous;

import choreo.auto.AutoFactory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Drivetrain.TranslateToBranch;
import frc.robot.commands.EndEffector.Coral.IntakeCoral;
import frc.robot.commands.EndEffector.Coral.ScoreL4;
import frc.robot.subsystems.Drivetrain.Swerve;
import frc.robot.subsystems.EndEffector.CoralIntake;
import frc.robot.subsystems.EndEffector.Elevator;
import frc.robot.subsystems.Vision.AprilTagCamera;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class TestAuto extends SequentialCommandGroup {
  /** Creates a new TestAuto. */
  private final AutoFactory m_Factory;
  public TestAuto(AutoFactory factory, CoralIntake CoralIntake, Swerve swerve, AprilTagCamera coralCam, Elevator elevator) {
    m_Factory = factory;
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    // Went way faster than Choreo velocity constraint
    addCommands(new SequentialCommandGroup(
      m_Factory.trajectoryCmd("TESTPATH1"),
      new IntakeCoral(CoralIntake),
      m_Factory.trajectoryCmd("TESTPATH2"),
      new TranslateToBranch(swerve, coralCam, true),
      new ScoreL4(elevator, CoralIntake)
    ));
        }

}
