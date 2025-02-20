package frc.robot.commands.Drivetrain;

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

  public enum RobotHeading {
    HEADING_1(0),
    HEADING_2(60),
    HEADING_3(120),
    HEADING_4(180),
    HEADING_5(240),
    HEADING_6(300);

    private final double angle;

    RobotHeading(double angle) {
      this.angle = angle;
    }

    public double getAngle() {
      return angle;
    }
  }

  // tag id to heading conversion
  Map<Integer, RobotHeading> tagToHeadingMap = new HashMap<>();

  private void initializeTagToHeadingMap() {
    tagToHeadingMap.put(7, RobotHeading.HEADING_1);
    tagToHeadingMap.put(8, RobotHeading.HEADING_2);
    tagToHeadingMap.put(9, RobotHeading.HEADING_3);
    tagToHeadingMap.put(10, RobotHeading.HEADING_4);
    tagToHeadingMap.put(11, RobotHeading.HEADING_5);
    tagToHeadingMap.put(6, RobotHeading.HEADING_6);
    tagToHeadingMap.put(18, RobotHeading.HEADING_1);
    tagToHeadingMap.put(17, RobotHeading.HEADING_2);
    tagToHeadingMap.put(22, RobotHeading.HEADING_3);
    tagToHeadingMap.put(21, RobotHeading.HEADING_4);
    tagToHeadingMap.put(20, RobotHeading.HEADING_5);
    tagToHeadingMap.put(19, RobotHeading.HEADING_6);
  }

  private final AprilTagCamera m_AprilTagCamera;
  private final Swerve m_Swerve;
  private final RotationController m_RotationController;
  private OptionalDouble headingGoal;

  public RotateToReef(Swerve swerve, AprilTagCamera camera) {
    m_Swerve = swerve;
    m_AprilTagCamera = camera;
    m_RotationController = new RotationController(swerve);
    initializeTagToHeadingMap();
    addRequirements(m_Swerve);
  }

  @Override
  public void initialize() {
    if (m_AprilTagCamera.getPrimaryTagID().isPresent()
        && tagToHeadingMap.containsKey(m_AprilTagCamera.getPrimaryTagID().getAsInt())) {
      headingGoal =
          OptionalDouble.of(
              tagToHeadingMap.get(m_AprilTagCamera.getPrimaryTagID().getAsInt()).getAngle());
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
