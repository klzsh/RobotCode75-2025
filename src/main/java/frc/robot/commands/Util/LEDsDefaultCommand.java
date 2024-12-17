// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Util;

import com.ctre.phoenix.led.StrobeAnimation;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Util.CANdleWrapper;
import frc.robot.subsystems.Util.CANdleWrapper.Section;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class LEDsDefaultCommand extends Command {
  /** Creates a new LEDsDefaultCommand. */
  private final CANdleWrapper m_Wrapper;

  public LEDsDefaultCommand(CANdleWrapper wrapper) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_Wrapper = wrapper;
    addRequirements(m_Wrapper);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_Wrapper.clearAnimation();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    /*
     * this is some basic modes for the lighting subsystem when the robot is in different states. this code does NOT modify LEDS when mechanism states change
     */
    if (DriverStation.isDisabled()) { // set team colors to breathe during disabled period
      m_Wrapper.sectionedBreathe(Color.kWhite, Section.kLeft, 250);
      m_Wrapper.sectionedBreathe(Color.kRed, Section.kRight, 250);
    } else if (DriverStation.isEStopped()) { // strobe red when EStopped
      m_Wrapper.setAnimation(new StrobeAnimation(255, 0, 0, 0, 0.80, 60, 9));
    } else if (DriverStation
        .isAutonomousEnabled()) { // set LEDS to green for auton (until note is obtained)
      m_Wrapper.setLEDSection(Color.kGreen, 9, 60);
    } else if (DriverStation.isTeleop()) { // set alliance color (if no note is in the robot)
      if (DriverStation.getAlliance().isPresent()) {
        if (DriverStation.getAlliance().get() == Alliance.Red) {
          m_Wrapper.setLEDSection(Color.kRed, 9, 60);
        } else {
          m_Wrapper.setLEDSection(Color.kBlue, 9, 60);
        }
      } else { // set team colors if alliance is not present for some reason
        m_Wrapper.setLEDSection(Color.kWhite, Section.kLeft);
        m_Wrapper.setLEDSection(Color.kRed, Section.kRight);
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_Wrapper.clearAnimation();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
