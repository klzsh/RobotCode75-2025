package frc.robot.commands.Drivetrain;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;

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

  // TODO: tune once new robot is made
  private final double translationStickMapValue = 1.5;

  public static final double translationJoystickExpo = 1.46;

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
    /* Get Values, Deadband */
    // add deadbands to prevent jittering on small stick inputs

    double[] state = Joysticks.processJoystick(translationSup, strafeSup, rotationSup);

    // you must negate the translation because FRC coordinates and Joystick axis values are
    // opposite to each other
    Translation2d translation2d =
        new Translation2d(state[0], state[1])
            .times(DrivetrainConstants.maxSpeed.in(MetersPerSecond));

    m_Swerve.drive(
        translation2d,
        state[2] * DrivetrainConstants.maxAngularVelocity.in(RadiansPerSecond),
        isOpenLoop,
        fieldRelative);
  }
}
