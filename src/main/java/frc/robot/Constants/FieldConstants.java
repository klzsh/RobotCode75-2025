package frc.robot.Constants;

import static java.util.Map.entry;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.lib.util.FieldPose;
import frc.lib.util.FieldPose.FieldElement;
import frc.lib.util.FieldPose.Offset;
import frc.lib.util.FieldPose.Side;
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
              new FieldPose(Side.RED, FieldElement.PROCESSOR, Offset.NONE),
              new Pose2d(0.0, 0.0, new Rotation2d(0.0))),
          // sideload
          entry(
              new FieldPose(Side.RED, FieldElement.TOPHPSTATION, Offset.SIDELOADMID),
              new Pose2d(0.0, 0.0, new Rotation2d(0.0))),
          entry(
              new FieldPose(Side.RED, FieldElement.TOPHPSTATION, Offset.LEFT),
              new Pose2d(0.0, 0.0, new Rotation2d(0.0))),
          entry(
              new FieldPose(Side.RED, FieldElement.TOPHPSTATION, Offset.RIGHT),
              new Pose2d(0.0, 0.0, new Rotation2d(0.0))),
          entry(
              new FieldPose(Side.RED, FieldElement.BOTTOMHPSTATION, Offset.SIDELOADMID),
              new Pose2d(0.0, 0.0, new Rotation2d(0.0))),
          entry(
              new FieldPose(Side.RED, FieldElement.BOTTOMHPSTATION, Offset.LEFT),
              new Pose2d(0.0, 0.0, new Rotation2d(0.0))),
          entry(
              new FieldPose(Side.RED, FieldElement.BOTTOMHPSTATION, Offset.RIGHT),
              new Pose2d(0.0, 0.0, new Rotation2d(0.0))),
          // frontload top station
          entry(
              new FieldPose(Side.RED, FieldElement.TOPHPSTATION, Offset.NONE),
              new Pose2d(0.0, 0.0, new Rotation2d(0.0))),
          entry(
              new FieldPose(Side.RED, FieldElement.TOPHPSTATION, Offset.LEFT1),
              new Pose2d(0.0, 0.0, new Rotation2d(0.0))),
          entry(
              new FieldPose(Side.RED, FieldElement.TOPHPSTATION, Offset.LEFT2),
              new Pose2d(0.0, 0.0, new Rotation2d(0.0))),
          entry(
              new FieldPose(Side.RED, FieldElement.TOPHPSTATION, Offset.LEFT3),
              new Pose2d(0.0, 0.0, new Rotation2d(0.0))),
          entry(
              new FieldPose(Side.RED, FieldElement.TOPHPSTATION, Offset.LEFT4),
              new Pose2d(0.0, 0.0, new Rotation2d(0.0))),
          entry(
              new FieldPose(Side.RED, FieldElement.TOPHPSTATION, Offset.RIGHT1),
              new Pose2d(0.0, 0.0, new Rotation2d(0.0))),
          entry(
              new FieldPose(Side.RED, FieldElement.TOPHPSTATION, Offset.RIGHT2),
              new Pose2d(0.0, 0.0, new Rotation2d(0.0))),
          entry(
              new FieldPose(Side.RED, FieldElement.TOPHPSTATION, Offset.RIGHT3),
              new Pose2d(0.0, 0.0, new Rotation2d(0.0))),
          entry(
              new FieldPose(Side.RED, FieldElement.TOPHPSTATION, Offset.RIGHT4),
              new Pose2d(0.0, 0.0, new Rotation2d(0.0))),
          // frontload bottom station
          entry(
              new FieldPose(Side.RED, FieldElement.BOTTOMHPSTATION, Offset.NONE),
              new Pose2d(0.0, 0.0, new Rotation2d(0.0))),
          entry(
              new FieldPose(Side.RED, FieldElement.BOTTOMHPSTATION, Offset.LEFT1),
              new Pose2d(0.0, 0.0, new Rotation2d(0.0))),
          entry(
              new FieldPose(Side.RED, FieldElement.BOTTOMHPSTATION, Offset.LEFT2),
              new Pose2d(0.0, 0.0, new Rotation2d(0.0))),
          entry(
              new FieldPose(Side.RED, FieldElement.BOTTOMHPSTATION, Offset.LEFT3),
              new Pose2d(0.0, 0.0, new Rotation2d(0.0))),
          entry(
              new FieldPose(Side.RED, FieldElement.BOTTOMHPSTATION, Offset.LEFT4),
              new Pose2d(0.0, 0.0, new Rotation2d(0.0))),
          entry(
              new FieldPose(Side.RED, FieldElement.BOTTOMHPSTATION, Offset.RIGHT1),
              new Pose2d(0.0, 0.0, new Rotation2d(0.0))),
          entry(
              new FieldPose(Side.RED, FieldElement.BOTTOMHPSTATION, Offset.RIGHT2),
              new Pose2d(0.0, 0.0, new Rotation2d(0.0))),
          entry(
              new FieldPose(Side.RED, FieldElement.BOTTOMHPSTATION, Offset.RIGHT3),
              new Pose2d(0.0, 0.0, new Rotation2d(0.0))),
          entry(
              new FieldPose(Side.RED, FieldElement.BOTTOMHPSTATION, Offset.RIGHT4),
              new Pose2d(0.0, 0.0, new Rotation2d(0.0))),
          // Reef positions
          entry(
              new FieldPose(Side.RED, FieldElement.REEFA, Offset.NONE),
              new Pose2d(0.0, 0.0, new Rotation2d(0.0))),
          entry(
              new FieldPose(Side.RED, FieldElement.REEFA, Offset.LEFT),
              new Pose2d(0.0, 0.0, new Rotation2d(0.0))),
          entry(
              new FieldPose(Side.RED, FieldElement.REEFA, Offset.RIGHT),
              new Pose2d(0.0, 0.0, new Rotation2d(0.0))),
          entry(
              new FieldPose(Side.RED, FieldElement.REEFB, Offset.NONE),
              new Pose2d(0.0, 0.0, new Rotation2d(0.0))),
          entry(
              new FieldPose(Side.RED, FieldElement.REEFB, Offset.LEFT),
              new Pose2d(0.0, 0.0, new Rotation2d(0.0))),
          entry(
              new FieldPose(Side.RED, FieldElement.REEFB, Offset.RIGHT),
              new Pose2d(0.0, 0.0, new Rotation2d(0.0))),
          entry(
              new FieldPose(Side.RED, FieldElement.REEFC, Offset.NONE),
              new Pose2d(0.0, 0.0, new Rotation2d(0.0))),
          entry(
              new FieldPose(Side.RED, FieldElement.REEFC, Offset.LEFT),
              new Pose2d(0.0, 0.0, new Rotation2d(0.0))),
          entry(
              new FieldPose(Side.RED, FieldElement.REEFC, Offset.RIGHT),
              new Pose2d(0.0, 0.0, new Rotation2d(0.0))),
          entry(
              new FieldPose(Side.RED, FieldElement.REEFD, Offset.NONE),
              new Pose2d(0.0, 0.0, new Rotation2d(0.0))),
          entry(
              new FieldPose(Side.RED, FieldElement.REEFD, Offset.LEFT),
              new Pose2d(0.0, 0.0, new Rotation2d(0.0))),
          entry(
              new FieldPose(Side.RED, FieldElement.REEFD, Offset.RIGHT),
              new Pose2d(0.0, 0.0, new Rotation2d(0.0))),
          entry(
              new FieldPose(Side.RED, FieldElement.REEFE, Offset.NONE),
              new Pose2d(0.0, 0.0, new Rotation2d(0.0))),
          entry(
              new FieldPose(Side.RED, FieldElement.REEFE, Offset.LEFT),
              new Pose2d(0.0, 0.0, new Rotation2d(0.0))),
          entry(
              new FieldPose(Side.RED, FieldElement.REEFE, Offset.RIGHT),
              new Pose2d(0.0, 0.0, new Rotation2d(0.0))),
          entry(
              new FieldPose(Side.RED, FieldElement.REEFF, Offset.NONE),
              new Pose2d(0.0, 0.0, new Rotation2d(0.0))),
          entry(
              new FieldPose(Side.RED, FieldElement.REEFF, Offset.LEFT),
              new Pose2d(0.0, 0.0, new Rotation2d(0.0))),
          entry(
              new FieldPose(Side.RED, FieldElement.REEFF, Offset.RIGHT),
              new Pose2d(0.0, 0.0, new Rotation2d(0.0))),
          entry(
              new FieldPose(Side.RED, FieldElement.REEFG, Offset.NONE),
              new Pose2d(0.0, 0.0, new Rotation2d(0.0))),
          entry(
              new FieldPose(Side.RED, FieldElement.REEFG, Offset.LEFT),
              new Pose2d(0.0, 0.0, new Rotation2d(0.0))),
          entry(
              new FieldPose(Side.RED, FieldElement.REEFG, Offset.RIGHT),
              new Pose2d(0.0, 0.0, new Rotation2d(0.0)))

          // TODO: Auto field flipping for blue side (or red side ig)
          );
}
