package frc.robot.commands.Drivetrain;

import static frc.robot.Constants.FieldConstants.*;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain.RotationController;
import frc.robot.subsystems.Drivetrain.Swerve;
import frc.robot.subsystems.Vision.AprilTagCamera;
import java.util.HashMap;
import java.util.Map;
import java.util.OptionalDouble;

public class RotateToReef extends Command {

  private final AprilTagCamera m_AprilTagCamera;
  private final Swerve m_Swerve;
  private final RotationController m_RotationController;
  private OptionalDouble headingGoal;

  public RotateToReef(Swerve swerve, AprilTagCamera camera) {
    m_Swerve = swerve;
    m_AprilTagCamera = camera;
    m_RotationController = new RotationController(swerve);
    addRequirements(m_Swerve);
  }

  @Override
  public void initialize() {
    if (m_AprilTagCamera.getPrimaryTagID().isPresent()
        && tagToHeadingMap.containsKey(m_AprilTagCamera.getPrimaryTagID().getAsInt())) {
      headingGoal =
          OptionalDouble.of(
              tagToHeadingMap.get(m_AprilTagCamera.getPrimaryTagID().getAsInt()));
    } else {
      headingGoal = OptionalDouble.empty();
    }
  }

  @Override
  public void execute() {
    if (headingGoal.isEmpty()) {
      return;
    }

    m_RotationController.update(new Rotation2d(headingGoal.getAsDouble()));

    m_Swerve.drive(new Translation2d(0, 0), m_RotationController.getOutput());
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return m_RotationController.atGoal();
  }
}
