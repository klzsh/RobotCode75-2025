// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drivetrain;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.subsystems.Drivetrain.Swerve;

public class XStance extends Command {
  private Swerve m_Swerve;

  /** Creates a new XStance. */
  // turns all wheels at a 45/-45 degree angle and locks the swerve drive in place
  public XStance(Swerve swerve) {
    m_Swerve = swerve;
    addRequirements(m_Swerve);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SwerveModuleState[] states =
        DrivetrainConstants.swerveKinematics.toSwerveModuleStates(
            new ChassisSpeeds(0, 0, 0), new Translation2d(0, 0));
    states[0].angle =
        Rotation2d.fromRadians(
            (Math.PI / 2)
                - Math.atan(
                    DrivetrainConstants.trackWidth.in(Meters)
                        / DrivetrainConstants.wheelBase.in(Meters)));
    states[1].angle =
        Rotation2d.fromRadians(
            (Math.PI / 2)
                + Math.atan(
                    DrivetrainConstants.trackWidth.in(Meters)
                        / DrivetrainConstants.wheelBase.in(Meters)));
    states[3].angle =
        Rotation2d.fromRadians(
            (Math.PI / 2)
                + Math.atan(
                    DrivetrainConstants.trackWidth.in(Meters)
                        / DrivetrainConstants.wheelBase.in(Meters)));
    states[2].angle =
        Rotation2d.fromRadians(
            (Math.PI / 2)
                - Math.atan(
                    DrivetrainConstants.trackWidth.in(Meters)
                        / DrivetrainConstants.wheelBase.in(Meters)));

    m_Swerve.setModuleStates(states);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_Swerve.stopModules();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
