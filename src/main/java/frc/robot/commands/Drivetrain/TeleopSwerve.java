package frc.robot.commands.Drivetrain;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.Constants.OIConstants;
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
    double translationVal =
        MathUtil.applyDeadband(
            translationSup.getAsDouble() * translationStickMapValue, OIConstants.stickDeadband);
    double strafeVal =
        MathUtil.applyDeadband(
            strafeSup.getAsDouble() * translationStickMapValue, OIConstants.stickDeadband);
    double rotationVal =
        MathUtil.applyDeadband(rotationSup.getAsDouble(), OIConstants.stickDeadband);

    translationVal =
        translationVal >= 0
            ? Math.pow(translationVal, translationJoystickExpo)
            : -1 * Math.pow(-translationVal, translationJoystickExpo);
    strafeVal =
        strafeVal >= 0
            ? Math.pow(strafeVal, translationJoystickExpo)
            : -1 * Math.pow(-strafeVal, translationJoystickExpo);

    // rotationVal = Math.pow(translationVal, translationJoystickExpo) *
    // Math.copySign(1.0,translationVal);
    // strafeVal = Math.pow(strafeVal, translationJoystickExpo) * Math.copySign(1.0,strafeVal);

    /*
     * make the translation to drive the robot
     * Multiply it by max speed as the drive command has units of meters per second
     */
    Translation2d translation2d =
        // you must negate the translation because FRC coordinates and Joystick axis values are
        // opposite to each other
        new Translation2d(translationVal, strafeVal)
            .times(DrivetrainConstants.maxSpeed.in(MetersPerSecond));

    // drive the robot. Multiple the rotation value by 0.5 to make the rotation easier to handle
    m_Swerve.drive(
        translation2d,
        rotationVal * DrivetrainConstants.maxAngularVelocity.in(RadiansPerSecond),
        isOpenLoop,
        fieldRelative);
  }
}
