// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Vision;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Drivetrain.Swerve;

public class OdometryWrapper extends SubsystemBase {
  private final Swerve m_Swerve;
  private final AprilTagCamera[] m_Cameras;

  /** Creates a new OdometryWrapper. */
  public OdometryWrapper(Swerve swerve, AprilTagCamera... cameras) {
    m_Swerve = swerve;
    m_Cameras = cameras;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    for (int i = 0; i < m_Cameras.length; i++) {
      m_Cameras[i].updateHeading(m_Swerve.getRotation2D());
      m_Cameras[i].updatePoseEstimator(m_Swerve.getPose());
      m_Swerve.updatePoseByVision(m_Cameras[i]);
    }
    // m_Swerve.updatePoseByVision(m_Cameras[i]);
    //

  }
}
