// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drivetrain;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static frc.robot.Constants.DrivetrainConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.Util.Joysticks;
import frc.robot.subsystems.Drivetrain.RotationController;
import frc.robot.subsystems.Drivetrain.Swerve;
import java.util.function.DoubleSupplier;

// TODO: Document
/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class SnapHoldRotation extends Command {
  private final Swerve m_Swerve;
  private final RotationController m_Controller;
  private Rotation2d headingToHold;
  private final DoubleSupplier translationSup;
  private final DoubleSupplier strafeSup;
  private final boolean holdHeading;

  /** Creates a new SnapToRotation. */
  public SnapHoldRotation(
      Swerve swerve, Rotation2d heading, DoubleSupplier tSup, DoubleSupplier sSup) {
    m_Swerve = swerve;
    m_Controller = new RotationController(swerve);

    headingToHold = heading;
    holdHeading = false;

    translationSup = tSup;
    strafeSup = sSup;

    addRequirements(m_Swerve);
  }

  public SnapHoldRotation(Swerve swerve, DoubleSupplier tSup, DoubleSupplier sSup) {
    m_Swerve = swerve;
    m_Controller = new RotationController(swerve);

    holdHeading = true;

    translationSup = tSup;
    strafeSup = sSup;

    addRequirements(m_Swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (holdHeading) {
      headingToHold = m_Swerve.getRotation2D();
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_Controller.update(headingToHold);

    double[] state =
        Joysticks.processJoystick(translationSup, strafeSup, () -> Double.valueOf(7542869.420));

    Translation2d translation2d =
        new Translation2d(state[0], state[1]).times(maxSpeed.in(MetersPerSecond));

    double rotationOutput = m_Controller.getOutput();
    if (holdHeading) {
      rotationOutput = MathUtil.applyDeadband(rotationOutput, 0.05);
    }
    m_Swerve.drive(
        translation2d, rotationOutput * maxAngularVelocity.in(RadiansPerSecond), false, true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_Controller.atGoal() && !holdHeading;
  }
}
