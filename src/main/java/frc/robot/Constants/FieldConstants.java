package frc.robot.Constants;

import static edu.wpi.first.units.Units.*;
import static java.util.Map.entry;

import edu.wpi.first.units.measure.Distance;
import frc.lib.util.FieldPose.FieldElement;
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

  public static final Map<Integer, Double> tagToHeadingMap =
      Map.ofEntries(
          entry(7, 0.0),
          entry(8, 60.0),
          entry(9, 120.0),
          entry(10, 180.0),
          entry(11, 240.0),
          entry(6, 300.0),
          entry(18, 0.0),
          entry(17, 60.0),
          entry(22, 120.0),
          entry(21, 180.0),
          entry(20, 240.0),
          entry(19, 300.0));

  public static final Map<FieldElement, Integer> blueTags =
      Map.ofEntries(
          entry(FieldElement.HT, 13),
          entry(FieldElement.HB, 14),
          entry(FieldElement.P, 16),
          entry(FieldElement.RL, 18),
          entry(FieldElement.RBL, 17),
          entry(FieldElement.RBR, 22),
          entry(FieldElement.RR, 21),
          entry(FieldElement.RTR, 20),
          entry(FieldElement.RTL, 19));

  public static final Map<FieldElement, Integer> redTags =
      Map.ofEntries(
          entry(FieldElement.HT, 1),
          entry(FieldElement.HB, 2),
          entry(FieldElement.P, 3),
          entry(FieldElement.RL, 7),
          entry(FieldElement.RBL, 8),
          entry(FieldElement.RBR, 9),
          entry(FieldElement.RR, 10),
          entry(FieldElement.RTR, 11),
          entry(FieldElement.RTL, 6));

  public static final Distance reefLeftPoseOffset = Meters.of(0.14);
  public static final Distance reefRightPoseOffset = Meters.of(0.5);
}
