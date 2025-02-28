// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drivetrain;

import static frc.robot.Constants.VisionConstants.heightThreshold;
import static frc.robot.Constants.VisionConstants.widthSetpoint;
import static frc.robot.Constants.VisionConstants.widthThreshold;
import static frc.robot.Constants.VisionConstants.widthTolerance;
import static frc.robot.Constants.VisionConstants.xSetpoint;
import static frc.robot.Constants.VisionConstants.xTolerance;

import java.util.List;

import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.dashboard.TunableNumber;
import frc.lib.util.CheckBounds;
import frc.lib.util.FieldPose;
import frc.robot.subsystems.Drivetrain.Swerve;
import frc.robot.subsystems.Vision.AprilTagCamera;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AlignToBranchColor extends Command {
  private final Swerve m_Swerve;
  private final AprilTagCamera m_BranchCam;
  private final PIDController m_XController = new PIDController(0, 0, 0);
  private final PIDController m_WidthController = new PIDController(0, 0, 0);
  private final TunableNumber xP;
  private final TunableNumber wP;

  /** Creates a new AlignToBranchColor. */
  public AlignToBranchColor(Swerve swerve, AprilTagCamera branchCam) {
    m_Swerve = swerve;
    m_BranchCam = branchCam;
    xP = new TunableNumber("AlignToBranchColor/xP", 0.1);
    wP = new TunableNumber("AlignToBranchColor/wP", 0.1);
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_XController.setSetpoint(xSetpoint);
    m_WidthController.setSetpoint(widthSetpoint);
    m_XController.setTolerance(xTolerance);
    m_WidthController.setTolerance(widthTolerance);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_BranchCam.getBestTarget().isEmpty()) {
      m_Swerve.setChassisSpeeds(new ChassisSpeeds(0, 0, 0));
      return;
    }
    m_XController.setP(xP.getNumber());
    m_WidthController.setP(wP.getNumber());
    PhotonTrackedTarget target = m_BranchCam.getBestTarget().get();
    List<TargetCorner> corners = target.getMinAreaRectCorners();
    TargetCorner topLeft = new TargetCorner(999, 999), bottomRight = new TargetCorner(0, 0);
    for (TargetCorner corner : corners) {
      if (corner.x < topLeft.x) {
        topLeft.x = corner.x;
      }
      if (corner.y < topLeft.y) {
        topLeft.y = corner.y;
      }
      if (corner.x > bottomRight.x) {
        bottomRight.x = corner.x;
      }
      if (corner.y > bottomRight.y) {
        bottomRight.y = corner.y;
      }
    }
    if (bottomRight.y - topLeft.y < heightThreshold || bottomRight.x - topLeft.x < widthThreshold) {
      m_Swerve.setChassisSpeeds(new ChassisSpeeds(0, 0, 0));
      return;
    }
    // robot relative, assuming heading is already aligned
    double xCommand = m_XController.calculate(
        (topLeft.x + bottomRight.x) / 2, xSetpoint);
    double yCommand = m_WidthController.calculate(
        bottomRight.x - topLeft.x, widthSetpoint);
    m_Swerve.setChassisSpeeds(new ChassisSpeeds(xCommand, yCommand, 0));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_Swerve.setChassisSpeeds(new ChassisSpeeds(0, 0, 0));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_XController.atSetpoint() && m_WidthController.atSetpoint();
  }
}
