package frc.lib.dashboard;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.Autonomous.AutoIntakeCoral;
import frc.robot.commands.Autonomous.AutoScoreL1;
import frc.robot.commands.Autonomous.AutoScoreL4;
import frc.robot.commands.Autonomous.AutoScoreProcessor;
import frc.robot.commands.Drivetrain.YoloBranchAlign;
import frc.robot.subsystems.Drivetrain.ChezyController;
import frc.robot.subsystems.Drivetrain.Swerve;
import frc.robot.subsystems.EndEffector.AlgaeIntake;
import frc.robot.subsystems.EndEffector.AlgaePivot;
import frc.robot.subsystems.EndEffector.CoralIntake;
import frc.robot.subsystems.EndEffector.Elevator;
import frc.robot.subsystems.Vision.ObjectDetetectorCamera;

/*
 * Return command based on a text code
 */

public class ActionFactory {
  private Swerve m_Swerve;
  private Elevator m_Elevator;
  private CoralIntake m_CoralIntake;
  private AlgaePivot m_AlgaePivot;
  private AlgaeIntake m_AlgaeIntake;
  private ChezyController m_ChezyController;
  private ObjectDetetectorCamera m_BranchCamera;

  public ActionFactory(
      Swerve swerve,
      Elevator elevator,
      CoralIntake coralIntake,
      AlgaePivot pivot,
      AlgaeIntake algaeIntake,
      ChezyController chezyController,
      ObjectDetetectorCamera branchCam) {
    m_Swerve = swerve;
    m_Elevator = elevator;
    m_CoralIntake = coralIntake;
    m_AlgaePivot = pivot;
    m_AlgaeIntake = algaeIntake;
    m_ChezyController = chezyController;
    m_BranchCamera = branchCam;
  }

  public Command getCommand(int action) {
    // 1 - L1, 2 - Dealgaefy, 3 - L4, 4 - Processor, 5 - Intake
    switch (action) {
      case 1:
        return new AutoScoreL1(m_Swerve, m_Elevator, m_CoralIntake);
        // case 2:
        //   return new AutoDealgaefy(
        //       m_Swerve, m_Elevator, m_AlgaeIntake, m_AlgaePivot, m_ChezyController);
      case 3:
        return new AutoScoreL4(m_Swerve, m_Elevator, m_CoralIntake);
      case 4:
        return new AutoScoreProcessor(m_Swerve, m_ChezyController, m_AlgaeIntake, m_AlgaePivot);
      case 5:
        return new AutoIntakeCoral(m_Swerve, m_CoralIntake);
      case 6:
        return new YoloBranchAlign(m_Swerve, m_BranchCamera, true);
    }
    return null;
  }

  public String getName(int action) {
    switch (action) {
      case 1:
        return "L1";
        // case 2:
        //   return "Dealgaefy";
      case 3:
        return "L4";
      case 4:
        return "Processor";
      case 5:
        return "Intake";
      case 6:
        return "YOLO";
    }
    return null;
  }
}
