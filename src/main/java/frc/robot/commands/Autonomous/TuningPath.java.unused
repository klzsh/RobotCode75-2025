// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autonomous;

import choreo.auto.AutoFactory;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TuningPath extends SequentialCommandGroup {
  /** Creates a new TuningPath. */
  private final AutoFactory m_Factory;

  public TuningPath(AutoFactory factory) {
    m_Factory = factory;
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(m_Factory.trajectoryCmd("TUNING_PATH_ROTATION"));
  }
}
