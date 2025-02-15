// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Constants;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;

/** Add your docs here. */
public class EndEffectorConstants {
  public static final double coralMotorGearRatio = 16.0;
  public static final double algaeMotorGearRatio = 25.0;
  public static final double pivotMotorGearRatio = (25.0 * 50.0) / 26.0;

  public static final AngularVelocity coralScoreSpeed = RotationsPerSecond.of(125);
  public static final AngularVelocity coralIntakeSpeed = RotationsPerSecond.of(100);
  public static final Angle coralRotationsAfterIntake = Rotations.of(0.5);

  public static final Angle pivotHomePosition = Rotations.of(9);
  // TODO: need to tune
  public static final Angle pivotGroundIntakePosition = Rotations.of(4);
  public static final Angle pivotDeAlgifyPosition = Rotations.of(7);

  public static final AngularVelocity algaeIntakeSpeed = RotationsPerSecond.of(150);
  public static final AngularVelocity algaeOutakeSpeed = RotationsPerSecond.of(500);
  public static final AngularVelocity algaeHoldSpeed = RotationsPerSecond.of(100);

  public static final double coralPositionDeadband = 0.5;
  public static final double algaePivotDeadband = 0.1;
}
