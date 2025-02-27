package frc.robot.subsystems.Drivetrain;

import static edu.wpi.first.units.Units.*;
import static frc.robot.Constants.DrivetrainConstants.ControllerConstants.*;
import static frc.robot.Constants.DrivetrainConstants.ControllerConstants.OdometryAlign.*;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Importance;
import edu.wpi.first.epilogue.Logged.Strategy;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.lib.dashboard.TunableNumber;

@Logged(name = "Pose Controller", strategy = Strategy.OPT_IN)
public class PoseAlignController {

  private final ProfiledPIDController translationController;
  private final RotationController thetaController;

  @Logged(importance = Importance.INFO)
  public ChassisSpeeds output;

  @Logged(importance = Importance.INFO)
  public Pose2d targetPose;

  @Logged(importance = Importance.INFO)
  public double distance2target;

  // Lock in the desired translation direction (in field coordinates) at reset.
  private double desiredTranslationAngleField;

  private final Swerve m_Swerve;

  private final TunableNumber[] translationPID = {
    new TunableNumber("AutoAlign/Ptr", xP),
    new TunableNumber("AutoAlign/Itr", xI),
    new TunableNumber("AutoAlign/Dtr", xD)
  };

  private final TunableNumber[] thetaPID = {
    new TunableNumber("AutoAlign/Pt", tP),
    new TunableNumber("AutoAlign/It", tI),
    new TunableNumber("AutoAlign/Dt", tD)
  };

  public PoseAlignController(Swerve swerve) {
    m_Swerve = swerve;
    translationController =
        new ProfiledPIDController(0, 0, 0, new TrapezoidProfile.Constraints(4, 4));
    thetaController = new RotationController(swerve);
    output = new ChassisSpeeds();

    translationController.setTolerance(toleranceTranslation);
    translationController.setConstraints(
        new TrapezoidProfile.Constraints(
            maxVelocity.in(MetersPerSecond), maxAcceleration.in(MetersPerSecondPerSecond)));
  }

  public double getVelocityFromSwerve() {
    ChassisSpeeds speeds = m_Swerve.getChassisSpeeds();
    return Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);
  }

  /**
   * Resets the controller. The desired translation direction (from current to target) is computed
   * in field coordinates and then locked in.
   */
  public void reset(Pose2d targetPose) {
    Pose2d currentPose = m_Swerve.getPose();
    Translation2d errorTranslation =
        targetPose.getTranslation().minus(currentPose.getTranslation());
    // Lock in the field-relative direction to the target.
    desiredTranslationAngleField = errorTranslation.getAngle().getRadians();
    double distance = errorTranslation.getNorm();
    translationController.reset(distance, getVelocityFromSwerve());
  }

  /**
   * Updates the controller. The translation command is computed in robot-relative coordinates by
   * converting the locked-in field direction using the current robot heading.
   */
  public ChassisSpeeds update(Pose2d currentPose, Pose2d targetPose) {
    // Update translation PID gains
    translationController.setPID(
        translationPID[0].getNumber(),
        translationPID[1].getNumber(),
        translationPID[2].getNumber());
    // (Optionally update theta PID gains as needed.)

    this.targetPose = targetPose;

    // Compute the current distance error (still in field coordinates).
    Translation2d errorTranslation =
        targetPose.getTranslation().minus(currentPose.getTranslation());
    distance2target = errorTranslation.getNorm();
    desiredTranslationAngleField = errorTranslation.getAngle().getRadians();

    // Calculate the translation velocity command.
    double velocityCommand = translationController.calculate(distance2target, 0);

    // Convert the locked field-relative desired direction into robot-relative coordinates.
    // (Subtract the current robot heading from the field angle.)
    Rotation2d currentRotation = currentPose.getRotation();
    double desiredTranslationAngleRobot =
        desiredTranslationAngleField - currentRotation.getRadians();

    // Compute the robot-relative x and y commands.
    double xCommand = velocityCommand * Math.cos(desiredTranslationAngleRobot);
    double yCommand = velocityCommand * Math.sin(desiredTranslationAngleRobot);

    xCommand =
        Math.min(
            maxVelocity.in(MetersPerSecond), Math.max(-maxVelocity.in(MetersPerSecond), xCommand));
    yCommand =
        Math.min(
            maxVelocity.in(MetersPerSecond), Math.max(-maxVelocity.in(MetersPerSecond), yCommand));

    // Compute the rotation error and update the rotation controller.
    // Rotation2d rotationError = targetPose.getRotation().minus(currentPose.getRotation());
    thetaController.update(
        targetPose.getRotation(), thetaPID[0].getNumber(), thetaPID[2].getNumber());
    double rotationalCommand = thetaController.getOutput();

    // Generate chassis speeds in the robot-relative coordinate system.
    output = new ChassisSpeeds(xCommand, yCommand, rotationalCommand);
    return output;
  }

  @Logged(importance = Importance.INFO)
  public boolean atGoal() {
    return translationController.atGoal() && thetaController.atGoal();
  }
}
