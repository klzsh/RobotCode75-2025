package frc.lib.dashboard;

import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

/**
 * Tunable Number takes in a number type and publishes an entry to a topic called "tuning" to be
 * used with advantagescope
 */
public class TunableNumber {

  private NetworkTableInstance instance = NetworkTableInstance.getDefault();
  private NetworkTable table = instance.getTable("Tuning");
  private DoubleEntry entry;

  public TunableNumber(String path, double number) {
    entry = table.getDoubleTopic(path).getEntry(0.0);
    entry.set(number);
  }

  public double getNumber() {
    return entry.get();
  }
}
