package frc.robot.Constants;

import static edu.wpi.first.units.Units.*;
import static java.util.Map.entry;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.lib.util.FieldPose;
import frc.lib.util.FieldPose.FieldElement;
import frc.lib.util.FieldPose.Offset;
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
          entry("a", ElevatorPositions.L3),
          entry("b", ElevatorPositions.L2),
          entry("c", ElevatorPositions.L3),
          entry("d", ElevatorPositions.L2),
          entry("e", ElevatorPositions.L3),
          entry("f", ElevatorPositions.L2),
          entry("A", ElevatorPositions.L3),
          entry("B", ElevatorPositions.L2),
          entry("C", ElevatorPositions.L3),
          entry("D", ElevatorPositions.L2),
          entry("E", ElevatorPositions.L3),
          entry("F", ElevatorPositions.L2));

  public static final Map<Integer, Double> tagToHeadingMap =
      Map.ofEntries(
          entry(6, 300.0),
          entry(7, 0.0),
          entry(8, 60.0),
          entry(9, 120.0),
          entry(10, 180.0),
          entry(11, 240.0),
          entry(17, 60.0),
          entry(18, 0.0),
          entry(19, 300.0),
          entry(20, 240.0),
          entry(21, 180.0),
          entry(22, 120.0));

  public static final Map<FieldElement, Integer> blueTags =
      Map.ofEntries(
          entry(FieldElement.A, 18),
          entry(FieldElement.B, 19),
          entry(FieldElement.C, 20),
          entry(FieldElement.D, 21),
          entry(FieldElement.E, 22),
          entry(FieldElement.F, 17),
          entry(FieldElement.HT, 13),
          entry(FieldElement.HB, 14),
          entry(FieldElement.P, 16));

  public static final Map<FieldElement, Integer> redTags =
      Map.ofEntries(
          entry(FieldElement.A, 7),
          entry(FieldElement.B, 6),
          entry(FieldElement.C, 11),
          entry(FieldElement.D, 10),
          entry(FieldElement.E, 9),
          entry(FieldElement.F, 8),
          entry(FieldElement.P, 3),
          entry(FieldElement.HT, 1),
          entry(FieldElement.HB, 2));

  public static final Map<FieldPose, Pose2d> fieldPose2d =
      Map.ofEntries(
          entry(
              new FieldPose(Alliance.Blue, FieldElement.HB, Offset.MID),
              new Pose2d(1.04, 1.02, Rotation2d.fromDegrees(54))),
          entry(
              new FieldPose(Alliance.Blue, FieldElement.HT, Offset.MID),
              new Pose2d(1.08, 7.04, Rotation2d.fromDegrees(-54))),
          entry(
              new FieldPose(Alliance.Red, FieldElement.HB, Offset.MID),
              new Pose2d(16.4, 7.08, Rotation2d.fromDegrees(234))),
          entry(
              new FieldPose(Alliance.Red, FieldElement.HT, Offset.MID),
              new Pose2d(16.4, .94, Rotation2d.fromDegrees(126))));

  public static final Distance reefLeftPoseOffset = Meters.of(0.11);
  public static final Distance reefRightPoseOffset = Meters.of(0.39);
}
