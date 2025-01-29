// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Constants;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Rotations;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;

/** Add your docs here. */
public class ElevatorConstants {
  // from the perspective of looking from the back of the robot forwards
  public static final int elevatorMotor1CANID = 41; // left
  public static final int elevatorMotor2CANID = 42; // right
  public static final int lowerLimitPort = 1;
  public static final int upperLimitPort = 0;
  public static final int backupLimitPort = 3;
  //   public static final int distanceSensorPort = 2;

  public static final Distance pulleyCircumference =
      Inches.of(1.751 * Math.PI * 2); // circumference of 22 teeth #25 WCP sprocket
  public static final double mechanismToMotorRatio =
      5.0 + (1.0 / 3.0); // 5 1/3 motor rotations = 1 shaft rotation

  public static final Distance inchesPerRotation =
      Inches.of(
          pulleyCircumference.in(Inches)
              / mechanismToMotorRatio); // 1 motor rotation = 2.068 inches of elevator movement
  // 8.7 inches from ground
  public static final Distance distanceBetweenElevatorZeroAndGround = Inches.of(8.7);

  // Distance from GROUND.
  public static final Angle algaeRemovalOffset = Rotations.of(0);
  public static final Angle l1Position = Rotations.of(2);
  public static final Angle l2Position = Rotations.of(7);
  public static final Angle l3Position = Rotations.of(15);
  public static final Angle l4Position = Rotations.of(25);
  public static final Angle homePosition = Rotations.of(0);
  public static final Angle processorPosition = Rotations.of(2);

  // velocity, acceleration, jerk
  public static double[] MotionMagicProfileUp = {70, 70, 1000};
  public static double[] MotionMagicProfileDown = {40, 40, 400};

  public static final Angle deadband = Rotations.of(0.25);

  public static final Angle maxDeviation = Rotations.of(0.5);
}
