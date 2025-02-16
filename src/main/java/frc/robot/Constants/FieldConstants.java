package frc.robot.Constants;

import static java.util.Map.entry;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.lib.util.FieldPose;
import frc.lib.util.FieldPose.FieldElement;
import frc.lib.util.FieldPose.Offset;
import java.util.Map;

public class FieldConstants {
  // so we need to hold poses for EVERY position
  // NOTE: These poses are where the ROBOT should be, NOT where the april tags are
  // Reef --> red/blue, 6 sides, left (coral), mid (algae), right (coral) --> 36 total
  // HP Station --> red/blue, 3 for sideloading + 7 for normal intake (-2 because of corner) --> 20
  // total
  // Processor --> red/blue, 1 position --> 2 total

  // TODO: change to entries bc im a dumdum
  public static final Map<FieldPose, Pose2d> fieldPoses =
      Map.ofEntries(
          // processor
          entry(
              new FieldPose(Alliance.Blue, FieldElement.P, Offset.MID),
              new Pose2d(0.0, 0.0, new Rotation2d(0.0))),
          // sideload
          entry(
              new FieldPose(Alliance.Blue, FieldElement.HT, Offset.MID),
              new Pose2d(0.0, 0.0, new Rotation2d(0.0))),
          entry(
              new FieldPose(Alliance.Blue, FieldElement.HT, Offset.LEFT),
              new Pose2d(0.0, 0.0, new Rotation2d(0.0))),
          entry(
              new FieldPose(Alliance.Blue, FieldElement.HT, Offset.RIGHT),
              new Pose2d(0.0, 0.0, new Rotation2d(0.0))),
          entry(
              new FieldPose(Alliance.Blue, FieldElement.HB, Offset.MID),
              new Pose2d(0.0, 0.0, new Rotation2d(0.0))),
          entry(
              new FieldPose(Alliance.Blue, FieldElement.HB, Offset.LEFT),
              new Pose2d(0.0, 0.0, new Rotation2d(0.0))),
          entry(
              new FieldPose(Alliance.Blue, FieldElement.HB, Offset.RIGHT),
              new Pose2d(0.0, 0.0, new Rotation2d(0.0))),
          // frontload top station
          // Reef positions
          entry(
              new FieldPose(Alliance.Blue, FieldElement.RL, Offset.MID),
              new Pose2d(5.15, 5.15, new Rotation2d(240.0))),
          entry(
              new FieldPose(Alliance.Blue, FieldElement.RL, Offset.LEFT),
              new Pose2d(0.0, 0.0, new Rotation2d(0.0))),
          entry(
              new FieldPose(Alliance.Blue, FieldElement.RL, Offset.RIGHT),
              new Pose2d(0.0, 0.0, new Rotation2d(0.0))),
          entry(
              new FieldPose(Alliance.Blue, FieldElement.RBL, Offset.MID),
              new Pose2d(0.0, 0.0, new Rotation2d(0.0))),
          entry(
              new FieldPose(Alliance.Blue, FieldElement.RBL, Offset.LEFT),
              new Pose2d(0.0, 0.0, new Rotation2d(0.0))),
          entry(
              new FieldPose(Alliance.Blue, FieldElement.RBL, Offset.RIGHT),
              new Pose2d(0.0, 0.0, new Rotation2d(0.0))),
          entry(
              new FieldPose(Alliance.Blue, FieldElement.RBR, Offset.MID),
              new Pose2d(0.0, 0.0, new Rotation2d(0.0))),
          entry(
              new FieldPose(Alliance.Blue, FieldElement.RBR, Offset.LEFT),
              new Pose2d(0.0, 0.0, new Rotation2d(0.0))),
          entry(
              new FieldPose(Alliance.Blue, FieldElement.RBR, Offset.RIGHT),
              new Pose2d(0.0, 0.0, new Rotation2d(0.0))),
          entry(
              new FieldPose(Alliance.Blue, FieldElement.RR, Offset.MID),
              new Pose2d(0.0, 0.0, new Rotation2d(0.0))),
          entry(
              new FieldPose(Alliance.Blue, FieldElement.RR, Offset.LEFT),
              new Pose2d(0.0, 0.0, new Rotation2d(0.0))),
          entry(
              new FieldPose(Alliance.Blue, FieldElement.RR, Offset.RIGHT),
              new Pose2d(0.0, 0.0, new Rotation2d(0.0))),
          entry(
              new FieldPose(Alliance.Blue, FieldElement.RTR, Offset.MID),
              new Pose2d(0.0, 0.0, new Rotation2d(0.0))),
          entry(
              new FieldPose(Alliance.Blue, FieldElement.RTR, Offset.LEFT),
              new Pose2d(0.0, 0.0, new Rotation2d(0.0))),
          entry(
              new FieldPose(Alliance.Blue, FieldElement.RTR, Offset.RIGHT),
              new Pose2d(0.0, 0.0, new Rotation2d(0.0))),
          entry(
              new FieldPose(Alliance.Blue, FieldElement.RTL, Offset.MID),
              new Pose2d(0.0, 0.0, new Rotation2d(0.0))),
          entry(
              new FieldPose(Alliance.Blue, FieldElement.RTL, Offset.LEFT),
              new Pose2d(0.0, 0.0, new Rotation2d(0.0))),
          entry(
              new FieldPose(Alliance.Blue, FieldElement.RTL, Offset.RIGHT),
              new Pose2d(0.0, 0.0, new Rotation2d(0.0)))

          // TODO: Auto field flipping for red side
          );
}
