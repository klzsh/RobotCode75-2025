package frc.robot.subsystems.Util;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Strategy;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycle;

@Logged(name = "Lidar sensor KP SUCKS", strategy = Strategy.OPT_IN)
public class LidarDistance {

  private double coralL1ThresholdIN = 36;
  DutyCycle sensorCycle;

  public LidarDistance() {
    sensorCycle = new DutyCycle(new DigitalInput(5));
  }

  private double getTimeNanoSeconds() {
    return sensorCycle.getHighTimeNanoseconds();
  }

  private double getDistanceMM() {
    return 4 * (getTimeNanoSeconds() - 1e6) / 1000;
  }

  @Logged(name = "distance inches")
  public double getDistanceIN() {
    return getDistanceMM() * 0.0393701;
  }

  public boolean isAligned() {
    return getDistanceIN() <= coralL1ThresholdIN;
  }
}
