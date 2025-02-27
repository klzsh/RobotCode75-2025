package frc.robot.commands.Drivetrain;

import static frc.robot.Constants.DrivetrainConstants.ControllerConstants.OdometryAlign.xP;
import static frc.robot.Constants.DrivetrainConstants.ControllerConstants.OdometryAlign.yP;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.dashboard.TunableNumber;
import frc.lib.util.CheckBounds;
import frc.lib.util.FieldPose;
import frc.robot.subsystems.Drivetrain.PoseAlignController;
import frc.robot.subsystems.Drivetrain.Swerve;
import frc.robot.subsystems.Vision.AprilTagCamera;
import java.util.List;
import java.util.Optional;
import org.photonvision.targeting.PhotonTrackedTarget;

public class TranslateToBranch extends Command {

  private final Swerve m_Swerve;
  private final AprilTagCamera m_Camera;
  private final boolean left;

  private final double finalYawSetpointLeft = 12.36;
  private final double finalPitchSetpointLeft = -6.5;
  private final double finalYawSetpointRight = -7.97;
  private final double finalPitchSetpointRight = -7.88;
  private final double finalYawSetpoint;
  private final double finalPitchSetpoint;

  private final PIDController xController;
  private final PIDController yController;
  private final TunableNumber xP;
  private final TunableNumber yP;
  private double xCommand;
  private double yCommand;

  private Optional<List<Integer>> visibleTagIDs;
  private int targetIDToFocus;

  private PoseAlignController fallbackController;
  private FieldPose fallbackPose;

  public TranslateToBranch(
      Swerve swerve,
      AprilTagCamera camera,
      boolean alignLeft,
      PoseAlignController fallbackController,
      FieldPose fallbackPose) {
    xP = new TunableNumber("Auto Align/Clapped X P", 0.1);
    yP = new TunableNumber("Auto Align/Clapped Y P", 0.1);

    this.fallbackController = fallbackController;
    this.fallbackPose = fallbackPose;

    m_Swerve = swerve;
    m_Camera = camera;
    left = alignLeft;

    xController = new PIDController(xP.getNumber(), 0, 0);
    yController = new PIDController(yP.getNumber(), 0, 0);

    if (left) {
      finalYawSetpoint = finalYawSetpointLeft;
      finalPitchSetpoint = finalPitchSetpointLeft;
    } else {
      finalYawSetpoint = finalYawSetpointRight;
      finalPitchSetpoint = finalPitchSetpointRight;
    }

    addRequirements(m_Swerve);
  }

  public double getLeftYawSetpoint(double currentPitch) {
    return MathUtil.clamp((2.47267 * (currentPitch) + 26.6736), -35, 35);
  }

  public double getRightYawSetpoint(double currentPitch) {
    return MathUtil.clamp((-5.50174 * (currentPitch) - 51.73407), -35, 35);
  }

  public Optional<Integer> getLargestAreaTargetID() {
    // get all visible tag IDs
    visibleTagIDs = m_Camera.getAllTagIds();
    // ensure at least one tag is visible
    if (visibleTagIDs.isPresent()) {
      double largestArea = 0;
      int largestAreaID = 0;
      // iterate through all visible tags and find the one with the largest area
      for (int id : visibleTagIDs.get()) {
        Optional<PhotonTrackedTarget> target = m_Camera.getTarget(id);
        if (target.isPresent() && target.get().getArea() > largestArea) {
          largestArea = target.get().getArea();
          largestAreaID = id;
        }
      }
      SmartDashboard.putNumber("Largest Area ID", largestAreaID);
      return Optional.of(largestAreaID);
    } else {
      end(true);

      return Optional.empty();
    }
  }

  @Override
  public void initialize() {
    // get the ID of the largest area target
    Optional<Integer> targetID = getLargestAreaTargetID();
    // if a target is visible, set the target ID to focus on to the largest area target
    if (targetID.isPresent()) {
      targetIDToFocus = targetID.get();
    }
  }

  @Override
  public void execute() {
    xController.setP(xP.getNumber());
    yController.setP(yP.getNumber());

    Optional<PhotonTrackedTarget> target = m_Camera.getTarget(targetIDToFocus);
    if (target.isPresent()) {
      double currentPitch = target.get().getPitch();
      double yawSetpoint =
          left ? getLeftYawSetpoint(currentPitch) : getRightYawSetpoint(currentPitch);
      xCommand =
          xController.calculate(
              Math.sin(Units.degreesToRadians(currentPitch)),
              Math.sin(Units.degreesToRadians(finalPitchSetpoint)));
      yCommand =
          yController.calculate(
              Math.sin(Units.degreesToRadians(target.get().getYaw())),
              Math.sin(Units.degreesToRadians(yawSetpoint)));

      m_Swerve.setChassisSpeeds(new ChassisSpeeds(xCommand, yCommand, 0));
    } else {
      ChassisSpeeds speeds =
          fallbackController.update(
              m_Swerve.getPose(), CheckBounds.getPose2DFromFieldPose(m_Swerve, fallbackPose));
      m_Swerve.setChassisSpeeds(speeds);
    }
  }

  @Override
  public boolean isFinished() {
    Optional<PhotonTrackedTarget> target = m_Camera.getTarget(targetIDToFocus);
    if (target.isPresent()) {
      return Math.abs(target.get().getPitch() - finalPitchSetpoint) < 1
          && Math.abs(target.get().getYaw() - finalYawSetpoint) < 1;
    }
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    // stop the robot
    m_Swerve.drive(new Translation2d(0, 0), 0);
  }
}
