package frc.robot.subsystems.Util;

import static edu.wpi.first.units.Units.Inches;
import static frc.robot.Constants.DrivetrainConstants.driveBusName;

import com.ctre.phoenix6.hardware.CANrange;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Importance;
import edu.wpi.first.epilogue.Logged.Strategy;
import edu.wpi.first.units.measure.Distance;

@Logged(name = "CAN Range Sensor", strategy = Strategy.OPT_IN)
public class CANRangeWrapper {
  private final CANrange m_CANrange;
  private final int deviceID = 6;
  double threshold = 0;

  public CANRangeWrapper(Distance threshold) {
    this.threshold = threshold.in(Inches);
    m_CANrange = new CANrange(deviceID, driveBusName);
  }

  @Logged(name = "CAN Range Distance", importance = Importance.DEBUG)
  public double getDistanceIN() {
    return m_CANrange.getDistance().refresh().getValue().in(Inches);
  }

  @Logged(name = "CAN Range is Aligned?", importance = Importance.DEBUG)
  public boolean isAligned() {
    return getDistanceIN() <= threshold;
  }
}
