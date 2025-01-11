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
      Inches.of(1.751 * Math.PI); // circumference of 22 teeth #25 WCP sprocket
  public static final double mechanismToMotorRatio = 3.0; // 3 motor rotations = 1 shaft rotation
  // TODO: sanity check this value
  public static final Distance inchesPerRotation =
      Inches.of(
          pulleyCircumference.in(Inches)
              * 2
              * mechanismToMotorRatio); // 1 motor rotation = 3 inches of elevator movement
  // TODO: find
  public static final Distance distanceBetweenElevatorZeroAndGround = Inches.of(0);

  // TODO: figure out
  // Distance from
  public static final Distance algaeRemovalOffset = Inches.of(0);
  public static final Distance l1Position = Inches.of(0);
  public static final Distance l2Position = Inches.of(0);
  public static final Distance l3Position = Inches.of(0);
  public static final Distance l4Position = Inches.of(0);
  public static final Distance homePosition = Inches.of(0);
  public static final Distance processorPosition = Inches.of(0);
  public static final Distance HPStationPosition = Inches.of(0);
}
