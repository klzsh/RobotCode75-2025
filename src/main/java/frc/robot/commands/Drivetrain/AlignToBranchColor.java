// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drivetrain;

import static frc.robot.Constants.VisionConstants.*;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.dashboard.TunableNumber;
import frc.robot.subsystems.Drivetrain.Swerve;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AlignToBranchColor extends Command {

  private final Swerve m_Swerve;
  private final NetworkTableInstance nt;
  private final NetworkTable ntTable;
  private final PIDController m_XController = new PIDController(0, 0, 0);
  // private final PIDController m_WidthController = new PIDController(0, 0, 0);
  private final TunableNumber xP;
  // private final TunableNumber wP;

  /** Creates a new AlignToBranchColor. */
  public AlignToBranchColor(Swerve swerve) {

    m_Swerve = swerve;
    xP = new TunableNumber("AlignToBranchColor/xP", 0.1);
    // wP = new TunableNumber("AlignToBranchColor/wP", 0.1);

    nt = NetworkTableInstance.getDefault();
    ntTable = nt.getTable("RaiderVision");
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_XController.setSetpoint(xSetpoint);
    // m_WidthController.setSetpoint(widthSetpoint);
    m_XController.setTolerance(xTolerance);
    // m_WidthController.setTolerance(widthTolerance);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_XController.setP(xP.getNumber());
    // m_WidthController.setP(wP.getNumber());

    String target = ntTable.getEntry("target").getString("None");
    if (target == "None") {
      m_Swerve.setChassisSpeeds(new ChassisSpeeds(0, 0, 0));
      return;
    }

    // if (bottomRight.y - topLeft.y < heightThreshold || bottomRight.x - topLeft.x <
    // widthThreshold) {
    //   m_Swerve.setChassisSpeeds(new ChassisSpeeds(0, 0, 0));
    //   return;
    // }
    // // robot relative, assuming heading is already aligned
    // double xCommand = m_XController.calculate((topLeft.x + bottomRight.x) / 2, xSetpoint);
    // double yCommand = m_WidthController.calculate(bottomRight.x - topLeft.x, widthSetpoint);
    // m_Swerve.setChassisSpeeds(new ChassisSpeeds(xCommand, yCommand, 0));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_Swerve.setChassisSpeeds(new ChassisSpeeds(0, 0, 0));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_XController.atSetpoint();
  }
}
