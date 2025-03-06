package frc.lib.dashboard;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.Autonomous.AutoDealgaefy;
import frc.robot.commands.Autonomous.AutoIntakeCoral;
import frc.robot.commands.Autonomous.AutoScoreL1;
import frc.robot.commands.Autonomous.AutoScoreL4;
import frc.robot.commands.Autonomous.AutoScoreProcessor;
import frc.robot.subsystems.Drivetrain.PoseAlignController;
import frc.robot.subsystems.Drivetrain.Swerve;
import frc.robot.subsystems.EndEffector.AlgaeIntake;
import frc.robot.subsystems.EndEffector.AlgaePivot;
import frc.robot.subsystems.EndEffector.CoralIntake;
import frc.robot.subsystems.EndEffector.Elevator;
import frc.robot.subsystems.Vision.AprilTagCamera;

public class ActionFactory {
  private Swerve m_Swerve;
  private Elevator m_Elevator;
  private CoralIntake m_CoralIntake;
  private AlgaePivot m_AlgaePivot;
  private AlgaeIntake m_AlgaeIntake;
  private PoseAlignController m_PoseAlignController;
  private AprilTagCamera m_LeftFacingCamera;
  private AprilTagCamera m_RightFacingCamera;

  public ActionFactory(
      Swerve swerve,
      Elevator elevator,
      CoralIntake coralIntake,
      AlgaePivot pivot,
      AlgaeIntake algaeIntake,
      PoseAlignController poseAlignController,
      AprilTagCamera leftCamera,
      AprilTagCamera rightCamera) {
    m_Swerve = swerve;
    m_Elevator = elevator;
    m_CoralIntake = coralIntake;
    m_AlgaePivot = pivot;
    m_AlgaeIntake = algaeIntake;
    m_PoseAlignController = poseAlignController;
    m_LeftFacingCamera = leftCamera;
    m_RightFacingCamera = rightCamera;
  }

  public Command getCommand(int action) {
    switch (action) {
      case 1:
        return new AutoScoreL1(m_Swerve, m_Elevator, m_CoralIntake, m_PoseAlignController);
      case 2:
        return new AutoDealgaefy(
            m_Swerve, m_Elevator, m_AlgaeIntake, m_AlgaePivot, m_PoseAlignController);
      case 3:
        return new AutoScoreL4(m_Swerve, m_Elevator, m_CoralIntake);
      case 4:
        return new AutoScoreProcessor(m_Swerve, m_PoseAlignController, m_AlgaeIntake, m_AlgaePivot);
      case 5:
        return new AutoIntakeCoral(m_Swerve, m_CoralIntake);
    }
    return null;
  }

  public String getName(int action) {
    switch (action) {
      case 1:
        return "L1";
      case 2:
        return "Dealgaefy";
      case 3:
        return "L4";
      case 4:
        return "Processor";
      case 5:
        return "Intake";
    }
    return null;
  }
}
