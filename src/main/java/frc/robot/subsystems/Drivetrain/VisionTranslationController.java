package frc.robot.subsystems.Drivetrain;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Importance;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.Timer;
import frc.lib.dashboard.TunableNumber;
import frc.robot.subsystems.Vision.AprilTagCamera;
import java.util.OptionalDouble;

public class VisionTranslationController {

  private final Swerve m_Swerve;

  private final PIDController xController;
  private final PIDController yController;

  private final AprilTagCamera m_CoralCamera;
  private final AprilTagCamera m_CenterCamera;

  private double lastSeenTagTime = 0.0;

  private OptionalDouble currentPitch;
  private OptionalDouble currentYaw;

  @Logged(name = "TranslateToBranch/XCommand", importance = Importance.CRITICAL)
  private double xCommand;

  @Logged(name = "TranslateToBranch/YCommand", importance = Importance.CRITICAL)
  private double yCommand;

  private TunableNumber[] xPID = {
    new TunableNumber("VisionController/xP", 0.05),
    new TunableNumber("VisionController/xI", 0),
    new TunableNumber("VisionController/xD", 0.0001)
  };

  private TunableNumber[] yPID = {
    new TunableNumber("VisionController/yP", 0.03),
    new TunableNumber("VisionController/yI", 0),
    new TunableNumber("VisionController/yD", 0.0001)
  };

  public VisionTranslationController(
      Swerve swerve, AprilTagCamera coralCamera, AprilTagCamera centerCamera) {
    m_Swerve = swerve;

    xController = new PIDController(xPID[0].getNumber(), xPID[1].getNumber(), xPID[2].getNumber());
    yController = new PIDController(yPID[0].getNumber(), yPID[1].getNumber(), yPID[2].getNumber());

    m_CoralCamera = coralCamera;
    m_CenterCamera = centerCamera;

    xController.setTolerance(2);
    yController.setTolerance(2);
  }

  // target: x is yaw, y is pitch
  public ChassisSpeeds update(Translation2d target, boolean alignLeft) {

    xController.setPID(xPID[0].getNumber(), xPID[1].getNumber(), xPID[2].getNumber());
    yController.setPID(yPID[0].getNumber(), yPID[1].getNumber(), yPID[2].getNumber());

    if (target == null) {
      System.out.println("Warning: No target");
      return new ChassisSpeeds(0, 0, 0);
    }

    xController.setSetpoint(target.getY());
    yController.setSetpoint(target.getX());

    AprilTagCamera camera = alignLeft ? m_CoralCamera : m_CenterCamera;

    if (!camera.hasTarget()) {
      return new ChassisSpeeds(xCommand, yCommand, 0);
    }

    lastSeenTagTime = Timer.getFPGATimestamp();

    // basically see if front-back we are < 6 inches away, and if we are then don't move forward
    // anymore
    int tagID = camera.getPrimaryTagID().getAsInt();
    Pose2d robotPose = m_Swerve.getPose();
    Pose2d tagPose =
        AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded)
            .getTagPose(tagID)
            .get()
            .toPose2d();
    Rotation2d tagHeading = tagPose.getRotation().rotateBy(Rotation2d.kCW_90deg);
    Pose2d reefPose2 =
        tagPose.transformBy(
            new Transform2d(
                Inches.of(3).times(-tagHeading.getCos()),
                Inches.of(3).times(-tagHeading.getSin()),
                Rotation2d.fromDegrees(0)));
    double l2 =
        Math.pow(reefPose2.getX() - tagPose.getX(), 2)
            + Math.pow(reefPose2.getY() - tagPose.getY(), 2);
    double dot =
        (robotPose.getX() - tagPose.getX()) * (robotPose.getX() - tagPose.getX())
            + (robotPose.getY() - tagPose.getY()) * (robotPose.getY() - tagPose.getY());
    double t = Math.max(0.0, Math.min(1.0, dot / l2));
    double projx = tagPose.getX() + t * (reefPose2.getX() - tagPose.getX());
    double projy = tagPose.getY() + t * (reefPose2.getY() - tagPose.getY());
    Distance dist =
        Meters.of(
            Math.sqrt(
                Math.pow(robotPose.getX() - projx, 2) + Math.pow(robotPose.getY() - projy, 2)));

    int xCommandMultiplier = 1;
    if (dist.in(Inches) < 6) {
      xCommandMultiplier = 0;
    }

    if (camera.getTarget(tagID).isPresent()) { // ensure tag to focus in view
      currentPitch = camera.getY(tagID); // Y is Pitch
      currentYaw = camera.getX(tagID); // X is Yaw

      System.out.println(currentPitch + " " + currentYaw);

      yCommand = yController.calculate(currentYaw.getAsDouble(), target.getX());
      xCommand =
          xController.calculate(currentPitch.getAsDouble(), target.getY()) * xCommandMultiplier;
    }

    return new ChassisSpeeds(xCommand, yCommand, 0); // robot relative
  }

  public boolean atGoal() {
    return xController.atSetpoint() && yController.atSetpoint();
  }
}
