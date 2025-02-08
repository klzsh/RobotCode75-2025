package frc.robot.Constants;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.Angle;

public final class ClimberConstants {
  // TODO: Change to actual values
  public static final int climberMotor1CANID = 46;
  public static final int climberMotor2CANID = 47;
  public static final int limitPort = 4;

  // if needed
  private static final double gearRatio = 39.6 / 1.0;

  // TODO: tune
  public static final Angle climbPosition = Rotations.of(0);
  public static final double climbDeadband = 0.5;
}
