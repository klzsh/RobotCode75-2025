package frc.robot.subsystems.Drivetrain;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Importance;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import frc.lib.dashboard.TunableNumber;
import frc.robot.subsystems.Vision.AprilTagCamera;
import java.util.OptionalDouble;

public class VisionTranslationController {

  private ProfiledPIDController xController;
  private ProfiledPIDController yController;

  private final AprilTagCamera m_CoralCamera;
  private final AprilTagCamera m_CenterCamera;

  private boolean alignLeft;

  private double lastSeenTagTime = 0.0;

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

  public VisionTranslationController(AprilTagCamera coralCamera, AprilTagCamera centerCamera) {
    m_CoralCamera = coralCamera;
    m_CenterCamera = centerCamera;

    xController.setTolerance(2);
    yController.setTolerance(2);
  }

  // target: x is yaw, y is pitch
  public ChassisSpeeds update(Translation2d target) {

    xController.setPID(xPID[0].getNumber(), xPID[1].getNumber(), xPID[2].getNumber());
    yController.setPID(yPID[0].getNumber(), yPID[1].getNumber(), yPID[2].getNumber());

    xController.setGoal(target.getY());
    yController.setGoal(target.getX());

    AprilTagCamera camera = alignLeft ? m_CoralCamera : m_CenterCamera;

    if (!camera.hasTarget()) {
      return new ChassisSpeeds(xCommand, yCommand, 0);
    }

    lastSeenTagTime = Timer.getFPGATimestamp();

    int tagID = camera.getPrimaryTagID().getAsInt();

    if (camera.getTarget(tagID).isPresent()) { // ensure tag to focus in view
      currentPitch = camera.getY(tagID); // Y is Pitch
      currentYaw = camera.getX(tagID); // X is Yaw

      yCommand = yController.calculate(currentYaw.getAsDouble(), target.getX());
      xCommand = xController.calculate(currentPitch.getAsDouble(), target.getY());
    }
    return new ChassisSpeeds(xCommand, yCommand, 0);
  }

  public boolean atGoal() {
    return xController.atGoal() && yController.atGoal();
  }
}
