package frc.robot.Constants;

import static edu.wpi.first.units.Units.*;
import static java.util.Map.entry;

import edu.wpi.first.units.measure.Distance;
import frc.robot.subsystems.EndEffector.Elevator.ElevatorPositions;
import java.util.Map;

public class FieldConstants {
  // so we need to hold poses for EVERY position
  // NOTE: These poses are where the ROBOT should be, NOT where the april tags are
  // Reef --> red/blue, 6 sides, left (coral), mid (algae), right (coral) --> 36 total
  // HP Station --> red/blue, 3 for sideloading + 7 for normal intake (-2 because of corner) --> 20
  // total
  // Processor --> red/blue, 1 position --> 2 total

  public static final Map<String, ElevatorPositions> algaeHeights =
      Map.ofEntries(
          entry("rl", ElevatorPositions.L3),
          entry("rbl", ElevatorPositions.L2),
          entry("rbr", ElevatorPositions.L3),
          entry("rr", ElevatorPositions.L2),
          entry("rtr", ElevatorPositions.L3),
          entry("rtl", ElevatorPositions.L2),
          entry("RL", ElevatorPositions.L3),
          entry("RBL", ElevatorPositions.L2),
          entry("RBR", ElevatorPositions.L3),
          entry("RR", ElevatorPositions.L2),
          entry("RTR", ElevatorPositions.L3),
          entry("RTL", ElevatorPositions.L2));

  public static final Distance reefLeftPoseOffset = Inches.of(4.5);
}
