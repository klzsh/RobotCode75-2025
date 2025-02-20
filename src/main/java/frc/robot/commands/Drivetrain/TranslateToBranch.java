package frc.robot.commands.Drivetrain;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Importance;
import edu.wpi.first.epilogue.Logged.Strategy;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.dashboard.TunableNumber;
import frc.robot.subsystems.Drivetrain.Swerve;
import frc.robot.subsystems.Vision.AprilTagCamera;
import java.util.Collections;
import java.util.Comparator;
import java.util.HashMap;
import java.util.Map;
import java.util.OptionalDouble;
import java.util.Set;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;


@Logged(name = "TranslateToBranch", strategy = Strategy.OPT_IN)
public class TranslateToBranch extends Command {

  private final Swerve m_Swerve;
  private boolean alignLeft;
  private VisionTranslationController2 visionController;

  public TranslateToBranch(Swerve swerve, boolean alignLeft, VisionTranslationController2 controller) {
    m_Swerve = swerve;
    this.alignLeft = alignLeft;

    visionController = controller;

    addRequirements(m_Swerve);
  }



  @Override
  public void initialize() {
    visionController.reset(alignLeft);
    }

    // tagIDToFocus is the tagID which relates to the nearest heading and the alliance



  @Override
  public void execute() {
    ChassisSpeeds speeds = m_controller.update(m_swerve.getPose(), targetPose);
    m_swerve.setChassisSpeeds(speeds);
    
  }



  @Override
  public void end(boolean interrupted) {
    
  }



  @Override
  public boolean isFinished() {
    // return false;
    return m_controller.atGoal();
  }
}
