package frc.robot.commands.Drivetrain;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.commands.Util.Joysticks;
import frc.robot.subsystems.Drivetrain.Swerve;
import java.util.function.DoubleSupplier;

/** This is the main command to drive the robot */
public class TeleopSwerve extends Command {
  private Swerve m_Swerve;
  private DoubleSupplier translationSup;
  private DoubleSupplier strafeSup;
  private DoubleSupplier rotationSup;
  private boolean isOpenLoop;
  private boolean fieldRelative;

  public TeleopSwerve(
      Swerve m_Swerve,
      DoubleSupplier translationSup,
      DoubleSupplier strafeSup,
      DoubleSupplier rotationSup,
      boolean isOpenLoop,
      boolean fieldRelative) {
    this.m_Swerve = m_Swerve;
    // get all values to drive the robot (x,y,z)
    this.translationSup = translationSup;
    this.strafeSup = strafeSup;
    this.rotationSup = rotationSup;
    this.isOpenLoop = isOpenLoop;
    this.fieldRelative = fieldRelative;

    addRequirements(m_Swerve);
  }

  @Override
  public void execute() {
    // translation, strafe, rotation is the order
    double[] DriverInput = Joysticks.processJoystick(translationSup, strafeSup, rotationSup);

    // you must negate the translation because FRC coordinates and Joystick axis values are
    // opposite to each other. This is done in robotContainer
    Translation2d translation2d =
        new Translation2d(DriverInput[0], DriverInput[1])
            .times(DrivetrainConstants.maxSpeed.in(MetersPerSecond));

    m_Swerve.drive(
        translation2d,
        DriverInput[2] * DrivetrainConstants.maxAngularVelocity.in(RadiansPerSecond),
        isOpenLoop,
        fieldRelative);
  }
}
