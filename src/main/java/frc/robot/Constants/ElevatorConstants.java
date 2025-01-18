// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Constants;

import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.units.measure.Distance;

/** Add your docs here. */
public class ElevatorConstants {
  // TODO: figure out
  public static final int elevatorMotor1CANID = 0;
  public static final int elevatorMotor2CANID = 0;

  public static final Distance pulleyCircumference =
      Inches.of(1.751 * Math.PI * 2); // circumference of 22 teeth #25 WCP sprocket
  public static final double mechanismToMotorRatio =
      5.0 + (1.0 / 3.0); // 5 1/3 motor rotations = 1 shaft rotation

  public static final Distance inchesPerRotation =
      Inches.of(
          pulleyCircumference.in(Inches)
              * mechanismToMotorRatio); // 1 motor rotation = 3 inches of elevator movement
  // 8.7 inches from ground
  public static final Distance distanceBetweenElevatorZeroAndGround = Inches.of(8.7);

  // TODO: figure out (placeholder values)
  // Distance from GROUND.
  public static final Distance algaeRemovalOffset = Inches.of(0);
  public static final Distance coralOffset = Inches.of(18.795);
  public static final Distance l1Position = Inches.of(8);
  public static final Distance l2Position = Inches.of(24);
  public static final Distance l3Position = Inches.of(48);
  public static final Distance l4Position = Inches.of(72);
  public static final Distance homePosition = Inches.of(0);
  public static final Distance processorPosition = Inches.of(7);

  // velocity, acceleration, jerk
  public static double[] MotionMagicProfileUp = {50, 200, 1200};
  public static double[] MotionMagicProfileDown = {25, 100, 1200};

  public static final Distance deadband = Inches.of(0.5);
}
