package frc.robot.commands.Drivetrain;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static frc.robot.Constants.DrivetrainConstants.*;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.Util.Joysticks;
import frc.robot.subsystems.Drivetrain.RotationController;
import frc.robot.subsystems.Drivetrain.Swerve;
import java.util.function.DoubleSupplier;

public class RotateToSimilarCoralStation extends Command {

  private final Swerve m_Swerve;
  private final RotationController m_RotationController;

  private DoubleSupplier m_TranslationSupplier;
  private DoubleSupplier m_StrafeSupplier;

  private double[] angles = {54, -54, 180 - 54, 180 + 54};

  private double targetAngle = 0;

  SwerveModuleState[] states = swerveKinematics.toSwerveModuleStates(new ChassisSpeeds(0, 0, 0));

  public RotateToSimilarCoralStation(
      Swerve swerve, DoubleSupplier translationSupplier, DoubleSupplier strafeSupplier) {
    m_Swerve = swerve;
    m_RotationController = new RotationController(swerve);

    m_TranslationSupplier = translationSupplier;
    m_StrafeSupplier = strafeSupplier;

    addRequirements(m_Swerve);
  }

  @Override
  public void initialize() {
    double minAngleDiff = 500;

    for (double angle : angles) {
      if (Math.abs(m_Swerve.getRotationDegrees() - angle) < minAngleDiff) {
        targetAngle = angle;
        minAngleDiff = Math.abs(m_Swerve.getRotationDegrees() - angle);
      }
    }
  }

  @Override
  public void execute() {
    double[] DriverInput =
        Joysticks.processJoystick(m_TranslationSupplier, m_StrafeSupplier, () -> 0);

    Translation2d translation2d = new Translation2d();

    if (DriverStation.getAlliance().get() == Alliance.Blue) {
      translation2d =
          new Translation2d(DriverInput[0], DriverInput[1]).times(maxSpeed.in(MetersPerSecond));
    } else {
      translation2d =
          new Translation2d(-DriverInput[0], -DriverInput[1]).times(maxSpeed.in(MetersPerSecond));
    }
    m_RotationController.update(Rotation2d.fromDegrees(targetAngle));

    double rotationOutput = 0;
    if (m_RotationController.atGoal()) {
      rotationOutput = 0;
    } else {
      rotationOutput = m_RotationController.getOutput();
    }

    m_Swerve.drive(translation2d, rotationOutput);
  }

  @Override
  public void end(boolean interrupted) {
    // xstance ideally i think
    if (Math.sqrt(
            Math.pow(m_Swerve.getChassisSpeeds().vxMetersPerSecond, 2)
                + Math.pow(m_Swerve.getChassisSpeeds().vyMetersPerSecond, 2))
        < .05) {
      m_Swerve.setChassisSpeeds(new ChassisSpeeds(0, 0, 0));
      states[0].angle = Rotation2d.fromDegrees(45);
      states[1].angle = Rotation2d.fromDegrees(315);
      states[3].angle = Rotation2d.fromDegrees(225);
      states[2].angle = Rotation2d.fromDegrees(135);
      for (SwerveModuleState state : states) {
        state.speedMetersPerSecond = 0;
      }

      m_Swerve.setModuleStates(states, true);
    }
  }

  @Override
  public boolean isFinished() {
    return false; // only ends on driver
  }
}
