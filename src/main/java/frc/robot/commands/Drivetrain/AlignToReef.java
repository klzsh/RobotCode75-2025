// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drivetrain;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.util.FieldPose;
import frc.lib.util.FieldPose.FieldElement;
import frc.lib.util.FieldPose.Offset;
import frc.lib.util.PeddieBounds;
import frc.robot.subsystems.Drivetrain.ChezyController;
import frc.robot.subsystems.Drivetrain.Swerve;
import frc.robot.subsystems.Vision.ObjectDetetectorCamera;
import frc.robot.subsystems.Vision.YoloController;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AlignToReef extends Command {
  private final Swerve m_Swerve;
  private final ChezyController m_ChezyController;
  private final YoloController m_YoloController;
  private final ObjectDetetectorCamera m_BranchCamera;
  private Offset m_Offset;
  private Pose2d targetPose = null;
  private ChassisSpeeds yoloSpeeds = null;

  /** Creates a new AlignToReef. */
  public AlignToReef(
      Swerve swerve,
      ChezyController chezyController,
      YoloController yoloController,
      ObjectDetetectorCamera branchCam,
      Offset offset) {
    m_Swerve = swerve;
    m_ChezyController = chezyController;
    m_YoloController = yoloController;
    m_BranchCamera = branchCam;
    m_Offset = offset;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_Swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    FieldElement fieldElement = PeddieBounds.getReefElement(m_Swerve);
    if (fieldElement != null) {
      targetPose =
          PeddieBounds.fieldElementToPose2d(
              m_Swerve, new FieldPose(DriverStation.getAlliance().get(), fieldElement, m_Offset));
      m_ChezyController.reset(targetPose);
    }
    m_YoloController.reset(true);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    ChassisSpeeds speeds = m_ChezyController.update(targetPose);
    // ChassisSpeeds speeds = m_ChezyController.update(new Pose2d(m_Swerve.getPose().getX(),
    // m_Swerve.getPose().getY(), targetPose.getRotation()));

    // chezy
    ChassisSpeeds chezySpeeds =
        ChassisSpeeds.fromFieldRelativeSpeeds(speeds, m_Swerve.getRotation2D());

    // robot relative y distance from center of branch
    double yOffset = Math.abs(targetPose.relativeTo(m_Swerve.getPose()).getY());

    m_BranchCamera.updateByUnreadResults();

    if (yOffset < 0.1 && m_BranchCamera.hasTargets() && m_ChezyController.isRotationFinished() && m_Offset != Offset.MID) {
      System.out.println("I have switched to YOLO");
      yoloSpeeds = m_YoloController.update();
      // add yolo speeds to chezy speeds
      chezySpeeds.vyMetersPerSecond = MathUtil.applyDeadband(yoloSpeeds.vyMetersPerSecond, 0.04);
      chezySpeeds.vxMetersPerSecond *= 0.5;
      chezySpeeds.vxMetersPerSecond = Math.max(chezySpeeds.vxMetersPerSecond, 0.1);
      // chezySpeeds.vxMetersPerSecond = 0.1;
    } else {
      System.out.println("Not using YOLO");
    }
    chezySpeeds.vxMetersPerSecond = MathUtil.clamp(chezySpeeds.vxMetersPerSecond, -2, 2);
    chezySpeeds.vyMetersPerSecond = MathUtil.clamp(chezySpeeds.vyMetersPerSecond, -2, 2);
    m_Swerve.setChassisSpeeds(chezySpeeds);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // m_Swerve.stopModules();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
    // return targetPose == null
    //     || (yoloSpeeds != null && Math.abs(yoloSpeeds.vyMetersPerSecond) < 0.02);
  }
}
