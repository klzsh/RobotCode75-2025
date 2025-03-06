// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drivetrain;

import static frc.robot.Constants.VisionConstants.*;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.dashboard.TunableNumber;
import frc.robot.subsystems.Drivetrain.Swerve;
import frc.robot.subsystems.Vision.ObjectDetetectorCamera;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AlignToBranchColor extends Command {

  private final Swerve m_Swerve;
  private final ObjectDetetectorCamera m_Camera;
  private final PIDController m_lrController = new PIDController(0, 0, 0);
  private final TunableNumber xP;
  private final TunableNumber xD;

  /** Creates a new AlignToBranchColor. */
  public AlignToBranchColor(Swerve swerve, ObjectDetetectorCamera camera) {
    m_Swerve = swerve;
    m_Camera = camera;
    xP = new TunableNumber("AlignToBranchColor/xP", 0.1);
    xD = new TunableNumber("AlignToBranchColor/xD", 0.0);
    // wP = new TunableNumber("AlignToBranchColor/wP", 0.1);
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_lrController.setSetpoint(xSetpoint);
    m_lrController.setTolerance(xTolerance);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_lrController.setP(xP.getNumber());
    m_lrController.setD(xD.getNumber());

    if (m_Camera.getTargetXFromCenter(0).isEmpty()) {
      m_Swerve.setChassisSpeeds(new ChassisSpeeds(0, 0, 0));
      return;
    }
    double currentX = m_Camera.getTargetXFromCenter(0).getAsDouble();

    // robot relative, assuming heading is already aligned
    double yCommand = m_lrController.calculate(currentX, xSetpoint);
    m_Swerve.setChassisSpeeds(new ChassisSpeeds(0, yCommand, 0));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_Swerve.stopModules();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_lrController.atSetpoint();
  }
}
