package frc.robot.Constants;

import static java.util.Map.entry;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
// import frc.robot.subsystems.Drivetrain.VisionController;
import frc.lib.util.FieldPose;
import frc.lib.util.FieldPose.FieldElement;
import frc.lib.util.FieldPose.Offset;
import frc.lib.util.FieldPose.Side;
import java.util.Map;

public class VisionConstants {
  public static final Matrix<N3, N1> moduleMatrix = MatBuilder.fill(Nat.N3(), Nat.N1(), 2, 2, .1);
  public static final Matrix<N3, N1> visionMatrix = MatBuilder.fill(Nat.N3(), Nat.N1(), 5, 5, 100);

  public static final Map<FieldPose, Translation2d> fieldPoseOffsets =
      // vision can't handle sideloading due to camera placement
      Map.ofEntries(
          // processor
          entry(
              new FieldPose(Side.RED, FieldElement.PROCESSOR, Offset.NONE),
              new Translation2d(0.0, 0.0)),

          // frontload top station AND bottom HP station
          entry(
              new FieldPose(Side.RED, FieldElement.TOPHPSTATION, Offset.NONE),
              new Translation2d(0.0, 0.0)),
          entry(
              new FieldPose(Side.RED, FieldElement.TOPHPSTATION, Offset.LEFT1),
              new Translation2d(0.0, 0.0)),
          entry(
              new FieldPose(Side.RED, FieldElement.TOPHPSTATION, Offset.LEFT2),
              new Translation2d(0.0, 0.0)),
          entry(
              new FieldPose(Side.RED, FieldElement.TOPHPSTATION, Offset.LEFT3),
              new Translation2d(0.0, 0.0)),
          entry(
              new FieldPose(Side.RED, FieldElement.TOPHPSTATION, Offset.LEFT4),
              new Translation2d(0.0, 0.0)),
          entry(
              new FieldPose(Side.RED, FieldElement.TOPHPSTATION, Offset.RIGHT1),
              new Translation2d(0.0, 0.0)),
          entry(
              new FieldPose(Side.RED, FieldElement.TOPHPSTATION, Offset.RIGHT2),
              new Translation2d(0.0, 0.0)),
          entry(
              new FieldPose(Side.RED, FieldElement.TOPHPSTATION, Offset.RIGHT3),
              new Translation2d(0.0, 0.0)),
          entry(
              new FieldPose(Side.RED, FieldElement.TOPHPSTATION, Offset.RIGHT4),
              new Translation2d(0.0, 0.0)),

          // Reef offsets (THESE SHOULD ALL BE THE SAME, THEY ARE JUST HERE FOR CONSISTENCY )
          entry(
              new FieldPose(Side.RED, FieldElement.REEFA, Offset.NONE),
              new Translation2d(0.0, 0.0)),
          entry(
              new FieldPose(Side.RED, FieldElement.REEFA, Offset.LEFT),
              new Translation2d(0.0, 0.0)),
          entry(
              new FieldPose(Side.RED, FieldElement.REEFA, Offset.RIGHT),
              new Translation2d(0.0, 0.0))

          // TODO: Auto field flipping for blue side (or red side ig)
          );

  public static double maxTimeUntilFallbackToOdometry = 1.0;
}
