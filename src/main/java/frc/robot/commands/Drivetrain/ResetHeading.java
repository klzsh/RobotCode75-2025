// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drivetrain;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain.Swerve;

/** resets the heading of the robot (should not be used often) */
public class ResetHeading extends Command {
  /** Creates a new ResetHeading. */
  private final Swerve m_Swerve;

  public ResetHeading(Swerve swerve) {
    m_Swerve = swerve;
    addRequirements(m_Swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // resets the gyro and pose based on the gyro
    m_Swerve.zeroGyro();
    m_Swerve.setPose(
        new Pose2d(m_Swerve.getPose().getX(), m_Swerve.getPose().getY(), new Rotation2d(0)));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    boolean resetted = false;
    // see if the pose has actually reset
    if (m_Swerve.getPose().getRotation().getDegrees() == 0) {
      resetted = true;
    } else {
      resetted = false;
    }
    return resetted;
  }
}
