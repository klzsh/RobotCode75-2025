// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drivetrain;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain.RotationController;
import frc.robot.subsystems.Drivetrain.Swerve;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class HoldRotation extends Command {
  private final Swerve m_Swerve;
  private final RotationController m_Controller;
  private final Rotation2d headingToHold;

  /** Creates a new HoldRotation. */
  public HoldRotation(Swerve swerve) {
    m_Swerve = swerve;
    m_Controller = new RotationController();
    headingToHold = m_Swerve.getRotation2D();
    addRequirements(m_Swerve);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_Controller.update(
        Degrees.of(m_Swerve.getRotation2D().getDegrees()), Degrees.of(headingToHold.getDegrees()));
    double rotationSetpoint = m_Controller.getSetpoint();
    ChassisSpeeds speeds =
        ChassisSpeeds.fromFieldRelativeSpeeds(
            m_Swerve.getDriverTranslationInput().getX(),
            m_Swerve.getDriverTranslationInput().getY(),
            rotationSetpoint,
            m_Swerve.getRotation2D());
    m_Swerve.setChassisSpeeds(speeds);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
