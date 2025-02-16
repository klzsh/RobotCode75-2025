package frc.robot.commands.Util;

import static frc.robot.Constants.OIConstants.*;

import edu.wpi.first.math.MathUtil;
import java.util.function.DoubleSupplier;

/**
 * Helper class to process joystick inputs. I wanted to use joystick expos to better control the
 * robot, so this is to simplify driver code
 */
public class Joysticks {
  public static double[] processJoystick(
      DoubleSupplier translationSup, DoubleSupplier strafeSup, DoubleSupplier rotationSup) {
    double translationVal =
        MathUtil.applyDeadband(
            translationSup.getAsDouble() * translationStickMapValue, stickDeadband);
    double strafeVal =
        MathUtil.applyDeadband(strafeSup.getAsDouble() * translationStickMapValue, stickDeadband);
    double rotationVal = MathUtil.applyDeadband(rotationSup.getAsDouble(), stickDeadband);

    translationVal =
        translationVal >= 0
            ? Math.pow(translationVal, translationJoystickExpo)
            : -1 * Math.pow(-translationVal, translationJoystickExpo);
    strafeVal =
        strafeVal >= 0
            ? Math.pow(strafeVal, translationJoystickExpo)
            : -1 * Math.pow(-strafeVal, translationJoystickExpo);

    /*
     * make the translation to drive the robot
     * Multiply it by max speed as the drive command has units of meters per second
     */
    return new double[] {translationVal, strafeVal, rotationVal};
  }
}
