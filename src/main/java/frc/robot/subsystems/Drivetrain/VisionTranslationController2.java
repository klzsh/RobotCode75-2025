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


public class VisionTrnaslationController2 {

  private PIDController xController;
  private PIDController yController;

  private final AprilTagCamera m_CoralCamera;
  private final AprilTagCamera m_CenterCamera;
  private final Swerve m_Swerve;
  private boolean alignLeft;

  private final double targetPitchLeft = -5;
  private final double targetPitchRight = 0.5;
  private final double targetYawLeft = 13;
  private final double targetYawRight = 0;

  private OptionalDouble currentPitch;
  private OptionalDouble currentYaw;

  @Logged(name = "TranslateToBranch/XCommand", importance = Importance.CRITICAL)
  private double xCommand;
  @Logged(name = "TranslateToBranch/YCommand", importance = Importance.CRITICAL)
  private double yCommand;

  private TunableNumber[] xPID = {
    new TunableNumber("TranslateToBranch/xP", 0.005),
    new TunableNumber("TranslateToBranch/xI", 0),
    new TunableNumber("TranslateToBranch/xD", 0.0001)
  };

  private TunableNumber[] yPID = {
    new TunableNumber("TranslateToBranch/yP", 0.001),
    new TunableNumber("TranslateToBranch/yI", 0),
    new TunableNumber("TranslateToBranch/yD", 0.0001)
  };

  private Set<Integer> correspondingTagIDs;
  private int tagIDToFocus;

  public VisionTrnaslationController2(Swerve swerve, AprilTagCamera coralCamera, AprilTagCamera centerCamera) {
    m_Swerve = swerve;
    m_CoralCamera = coralCamera;
    m_CenterCamera = centerCamera;
  }


  public void reset(boolean alignLeft) {
    this.alignLeft = alignLeft
    if (alignLeft) {
      xController.setSetpoint(targetPitchLeft);
      yController.setSetpoint(targetYawLeft);
    } else {
      xController.setSetpoint(targetPitchRight);
      yController.setSetpoint(targetPitchLeft);
    }
    xController.setTolerance(5);
    yController.setTolerance(5);

    // Determine the tagIDToFocus based on the alliance
    Alliance alliance = DriverStation.getAlliance().get();
    if (alliance == Alliance.Red) {
      tagIDToFocus = correspondingTagIDs.stream()
        .filter(id -> id >= 6 && id <= 11)
        .findFirst()
        .orElseThrow(() -> new IllegalStateException("No valid tag ID found for Red alliance"));
    } else if (alliance == Alliance.Blue) {
      tagIDToFocus = correspondingTagIDs.stream()
        .filter(id -> id >= 17 && id <= 22)
        .findFirst()
        .orElseThrow(() -> new IllegalStateException("No valid tag ID found for Blue alliance"));
    } else {
      throw new IllegalStateException("Unknown alliance: " + alliance);
    }

    // tagIDToFocus is the tagID which relates to the nearest heading and the alliance
  }


  public void update() {
    xController.setPID(xPID[0].getNumber(), xPID[1].getNumber(), xPID[2].getNumber());
    yController.setPID(yPID[0].getNumber(), yPID[1].getNumber(), yPID[2].getNumber());

    AprilTagCamera camera = m_CoralCamera ? alignleft : m_CenterCamera;
    
    if (camera.getTarget(tagIDToFocus).isPresent()) { // ensure tag to focus in view
      currentPitch = camera.getY(tagIDToFocus); // Y is Pitch
      currentYaw = camera.getX(tagIDToFocus); // X is Yaw
      if (alignLeft) {
        xCommand = xController.calculate(currentPitch.getAsDouble(), targetPitchLeft);
        yCommand = yController.calculate(currentYaw.getAsDouble(), targetYawLeft);
      } else {
        xCommand = xController.calculate(currentPitch.getAsDouble(), targetPitchRight);
        yCommand = yController.calculate(currentYaw.getAsDouble(), targetYawRight);
      }

      // m_Swerve.drive(new Translation2d(xCommand, yCommand), 0); // needs to be robot relative
      m_Swerve.setRobotRelative(new ChassisSpeeds(xCommand, yCommand, 0));
    }
  }

}
