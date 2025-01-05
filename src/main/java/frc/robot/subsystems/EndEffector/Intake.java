// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.EndEffector;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

/*
 * The Intake serves a dual purpose. It intakes both Coral and Algae.
 *
 * The Coral taken in by a set of rollers driven by a (tentative) kraken x60.
 * There is a beam break sensor which will detect if there is a coral in the intake
 *
 * The algae intake also has two rollers driven by a (tentative) kraken X60.
 * The current applied to the stator will tell us when the algae is fully in the intake.
 *   There is no limit switch for the algae.
 */
public class Intake extends SubsystemBase {
  /** Creates a new intake. */
  public Intake() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
