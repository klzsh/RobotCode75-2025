package frc.robot.subsystems.Drivetrain;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Importance;
import edu.wpi.first.epilogue.Logged.Strategy;
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

@Logged(name = "Vision Controller", strategy = Strategy.OPT_IN)
public class VisionTranslationController {

  private final Swerve m_Swerve;

  private final PIDController xController;
  private final PIDController yController;

  private final AprilTagCamera m_RightFacingCamera;
  private final AprilTagCamera m_LeftFacingCamera;

  @Logged(importance = Importance.INFO)
  private double lastSeenTagTime = 0.0;

  @Logged(importance = Importance.INFO)
  private OptionalDouble currentPitch;

  @Logged(importance = Importance.INFO)
  private OptionalDouble currentYaw;

  @Logged(name = "XOutput", importance = Importance.INFO)
  private double xCommand;

  @Logged(name = "YOutput", importance = Importance.INFO)
  private double yCommand;

  @Logged(importance = Importance.INFO)
  private ChassisSpeeds output;

  private TunableNumber[] xPID_Coral = {
    new TunableNumber("VisionController/CoralCamera/xP", 0.05),
    new TunableNumber("VisionController/CoralCamera/xI", 0),
    new TunableNumber("VisionController/CoralCamera/xD", 0.0001)
  };

  private TunableNumber[] yPID_Coral = {
    new TunableNumber("VisionController/CoralCamera/yP", 0.03),
    new TunableNumber("VisionController/CoralCamera/yI", 0),
    new TunableNumber("VisionController/CoralCamera/yD", 0.0001)
  };

  private TunableNumber[] xPID_Center = {
    new TunableNumber("VisionController/CenterCamera/xP", 0.0045),
    new TunableNumber("VisionController/CenterCamera/xI", 0),
    new TunableNumber("VisionController/CenterCamera/xD", 0)
  };

  private TunableNumber[] yPID_Center = {
    new TunableNumber("VisionController/CenterCamera/yP", 0.014),
    new TunableNumber("VisionController/CenterCamera/yI", 0),
    new TunableNumber("VisionController/CenterCamera/yD", 0.0001)
  };

  public VisionTranslationController(
      Swerve swerve, AprilTagCamera leftFacingCamera, AprilTagCamera rightFacingCamera) {
    m_Swerve = swerve;

    xController = new PIDController(0, 0, 0);
    yController = new PIDController(0, 0, 0);

    m_RightFacingCamera = rightFacingCamera;
    m_LeftFacingCamera = leftFacingCamera;

    xController.setTolerance(2);
    yController.setTolerance(2);

    xController.enableContinuousInput(-90, 90);
    yController.enableContinuousInput(-90, 90);
  }

  // target: x is yaw, y is pitch
  public ChassisSpeeds update(Translation2d target, boolean alignLeft) {

    if (target == null) {
      System.out.println("Warning: No target");
      output = new ChassisSpeeds(0, 0, 0);
      return new ChassisSpeeds(0, 0, 0);
    }

    xController.setSetpoint(target.getY());
    yController.setSetpoint(target.getX());

    AprilTagCamera camera = alignLeft ? m_RightFacingCamera : m_LeftFacingCamera;

    if (!camera.hasTarget()) {
      output = new ChassisSpeeds(xCommand, yCommand, 0);
      return new ChassisSpeeds(xCommand, yCommand, 0);
    }

    if (alignLeft) {
      xController.setTolerance(1);
      yController.setTolerance(1);
      xController.setPID(
          xPID_Coral[0].getNumber(), xPID_Coral[1].getNumber(), xPID_Coral[2].getNumber());
      yController.setPID(
          yPID_Coral[0].getNumber(), yPID_Coral[1].getNumber(), yPID_Coral[2].getNumber());
    } else {
      xController.setTolerance(1);
      yController.setTolerance(5);
      xController.setPID(
          xPID_Center[0].getNumber(), xPID_Center[1].getNumber(), xPID_Center[2].getNumber());
      yController.setPID(
          yPID_Center[0].getNumber(), yPID_Center[1].getNumber(), yPID_Center[2].getNumber());
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

    double xCommandMultiplier = 1;
    if (dist.in(Inches) < 19) {
      xCommandMultiplier = 0.2;
    }

    if (camera.getTarget(tagID).isPresent()) { // ensure tag to focus in view
      currentPitch = camera.getY(tagID); // Y is Pitch
      currentYaw = camera.getX(tagID); // X is Yaw

      yCommand = yController.calculate(currentYaw.getAsDouble(), target.getX());
      // if (currentYaw.getAsDouble() < 0) {
      //   yCommand *= -1;
      // }
      xCommand =
          xController.calculate(currentPitch.getAsDouble(), target.getY()) * xCommandMultiplier;
    }
    output = new ChassisSpeeds(xCommand, yCommand, 0);
    return new ChassisSpeeds(xCommand, yCommand, 0); // robot relative
  }

  @Logged(importance = Importance.INFO)
  public boolean atGoal() {
    return xController.atSetpoint() && yController.atSetpoint();
  }
}
