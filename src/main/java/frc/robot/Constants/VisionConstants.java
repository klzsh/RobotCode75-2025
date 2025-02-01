package frc.robot.Constants;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.measure.Angle;
import frc.robot.subsystems.Drivetrain.VisionController;

import java.util.HashMap;
import java.util.Map;

import static edu.wpi.first.units.Units.Rotations;

public class VisionConstants {
  public static final Matrix<N3, N1> moduleMatrix = MatBuilder.fill(Nat.N3(), Nat.N1(), 2, 2, .1);
  public static final Matrix<N3, N1> visionMatrix = MatBuilder.fill(Nat.N3(), Nat.N1(), 5, 5, 100);

  public static final Translation2d[] REEF_LEFT_OFFSET = {
      new Translation2d(0, 0),
      new Translation2d(0, 0),
      new Translation2d(0, 0)
  };
  public static final Translation2d[] REEF_RIGHT_OFFSET = {
          new Translation2d(0, 0),
          new Translation2d(0, 0),
          new Translation2d(0, 0)
  };
  public static final Translation2d[] HP_LEFT_OFFSET = {
          new Translation2d(0, 0),
          new Translation2d(0, 0),
          new Translation2d(0, 0)
  };
  public static final Translation2d[] HP_CENTER_OFFSET = {
          new Translation2d(0, 0),
          new Translation2d(0, 0),
          new Translation2d(0, 0)
  };
  public static final Translation2d[] HP_RIGHT_OFFSET = {
          new Translation2d(0, 0),
          new Translation2d(0, 0),
          new Translation2d(0, 0)
  };

  public static final Translation2d[] aprilTagOffsets = {
          new Translation2d(0, 0),
          new Translation2d(0, 0)
  };



  public static final Map<Integer, Map<VisionController.AlignTargets, Pose2d>> aprilTagPoses = Map.ofEntries(
      Map.entry(0, Map.of(
          VisionController.AlignTargets.REEF_LEFT, new Pose2d(0, 0, Rotation2d.fromDegrees(0))
      )),
      Map.entry(1, Map.of(
          VisionController.AlignTargets.REEF_LEFT, new Pose2d(0, 0, Rotation2d.fromDegrees(0))
      )),
      Map.entry(2, Map.of(
              null, 
      )),
  );

  public static double maxTimeUntilFallbackToOdometry = 1.0;
}