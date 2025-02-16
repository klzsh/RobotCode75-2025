// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autonomous;

import static frc.robot.Constants.FieldConstants.fieldPoses;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.lib.util.FieldPose;
import frc.lib.util.FieldPose.Offset;
import frc.robot.commands.Drivetrain.DriveToPose;
import frc.robot.subsystems.Drivetrain.Swerve;
import frc.robot.subsystems.EndEffector.AlgaeIntake;
import frc.robot.subsystems.EndEffector.AlgaePivot;
import frc.robot.subsystems.EndEffector.Elevator;
import frc.robot.subsystems.Vision.AprilTagCamera;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoDealgaefy extends SequentialCommandGroup {
  /** Creates a new AutoDealgaefy. */
  public AutoDealgaefy(
      Swerve swerve,
      Elevator elevator,
      AlgaeIntake intake,
      AlgaePivot pivot,
      AprilTagCamera centerCamera,
      String reefPoint) {
    addRequirements(swerve, elevator, intake, pivot);
    addCommands(
        new ParallelCommandGroup(
            // new VisionAlign(swerve, centerCamera, ),
            new DriveToPose(
                swerve,
                fieldPoses.get(
                    new FieldPose(
                        DriverStation.getAlliance().get(),
                        FieldPose.fromString(reefPoint),
                        Offset.MID)),
                false)));
  }
}
