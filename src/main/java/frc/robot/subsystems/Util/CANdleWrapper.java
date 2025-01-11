// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Util;

import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.CANdle;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CANdleWrapper extends SubsystemBase {
  /** Creates a new CANdleWrapper. */
  // TODO: rework for 2025 robot
  private final CANdle m_Candle;

  public enum Section {
    kLeft,
    kRight
  }

  public CANdleWrapper() {
    m_Candle = new CANdle(4);
    m_Candle.configBrightnessScalar(0.5);
  }

  private void setLeftHalf(Color color) {
    m_Candle.setLEDs(
        (int) (color.red * 255), (int) (color.green * 255), (int) (color.blue * 255), 0, 9, 30);
  }

  private void setRightHalf(Color color) {
    m_Candle.setLEDs(
        (int) (color.red * 255), (int) (color.green * 255), (int) (color.blue * 255), 0, 39, 30);
  }

  public void setAnimation(Animation animation) {
    m_Candle.animate(animation);
  }

  public void clearAnimation() {
    m_Candle.clearAnimation(0);
  }

  public void sectionedBreathe(Color color, Section section, double breathePeriod) {
    double breatheOffset = Timer.getFPGATimestamp() * 1000 % (2 * breathePeriod);

    // Breathe in from [0, interval]
    // Breathe out from [interval, 2*interval]
    // abs() flips direction of breathing
    breatheOffset = Math.abs(breatheOffset - breathePeriod);

    setLEDSection(
        new Color(
            color.red * breatheOffset / breathePeriod,
            color.green * breatheOffset / breathePeriod,
            color.blue * breatheOffset / breathePeriod),
        section);
  }

  public void sectionedBlink(Color color, Section section, double blinkPeriod) {
    double blinkOffset = Timer.getFPGATimestamp() * 1000 % (2 * blinkPeriod);

    // From [0, interval], off
    if (blinkOffset > blinkPeriod) {
      setLEDSection(color, section);
    }
    // From [interval, 2*interval], on
    else setLEDSection(Color.kBlack, section);
  }

  public void setLEDSection(Color color, int startIndex, int numLeds) {
    m_Candle.setLEDs(
        (int) (color.red * 255),
        (int) (color.green * 255),
        (int) (color.blue * 255),
        0,
        startIndex,
        numLeds);
  }

  public void setLEDSection(Color color, Section section) {
    if (section == Section.kLeft) {
      setLeftHalf(color);
    } else if (section == Section.kRight) {
      setRightHalf(color);
    }
  }

  public void clearSection(Section section) {
    setLEDSection(Color.kBlack, section);
  }

  public void off() {
    m_Candle.setLEDs(0, 0, 0, 0, 0, 69);
  }

  @Override
  public void periodic() {
    sectionedBlink(Color.kRed, Section.kLeft, 100);
    // This method will be called once per scheduler run
  }
}
