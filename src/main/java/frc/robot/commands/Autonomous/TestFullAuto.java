// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autonomous;

import choreo.auto.AutoFactory;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Drivetrain.Swerve;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TestFullAuto extends SequentialCommandGroup {
  private final Swerve m_Swerve;
  private final AutoFactory m_AutoFactory;
  private final String routineName = "Simple3Piece";

  public TestFullAuto(Swerve swerve, AutoFactory autoFactory) {
      m_AutoFactory = autoFactory;
      m_Swerve = swerve;

      addCommands(
          m_AutoFactory.resetOdometry(routineName, 0),
          m_AutoFactory.trajectoryCmd(routineName, 0), // start to clapped 2 left right branch
          new WaitCommand(2), // wait instead of up, place , down
          m_AutoFactory.trajectoryCmd(routineName, 1), // clapped 2 left right branch to coral station
          new WaitCommand(.25), // wait instead of intake
          m_AutoFactory.trajectoryCmd(routineName, 2), // coral station to big 2 left left branch
          new WaitCommand(2), // wait instead of up, place , down
          m_AutoFactory.trajectoryCmd(routineName, 3), // big 2 left left branch to coral station
          new WaitCommand(.25), // wait instead of intake
          m_AutoFactory.trajectoryCmd(routineName, 4), // coral station to big 2 right right branch
          new WaitCommand(2) // wait instead of up, place , down
      );
  }
}
