package frc.lib.dashboard;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import java.util.concurrent.atomic.AtomicReference;

/**
 * Tunable Number takes in a number type and publishes an entry to a topic called "tuning" to be
 * used with advantagescope
 */
public class TunableNumber<T extends Number> {

  private AtomicReference<T> number;
  private NetworkTableInstance instance = NetworkTableInstance.getDefault();
  private NetworkTable table = instance.getTable("Tuning");
  private GenericEntry entry;

  public TunableNumber(T number) {
    this.number.set(number);
  }

  public void publishNumber(T number) {}

  public T getNumber() {
    return null;
  }
}
