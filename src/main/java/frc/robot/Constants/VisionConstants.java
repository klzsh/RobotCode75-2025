package frc.robot.Constants;

import static java.util.Map.entry;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
// import frc.robot.subsystems.Drivetrain.VisionController;
import frc.lib.util.FieldPose;
import frc.lib.util.FieldPose.FieldElement;
import frc.lib.util.FieldPose.Offset;
import java.util.Map;

public class VisionConstants {
  public static final Matrix<N3, N1> moduleMatrix = MatBuilder.fill(Nat.N3(), Nat.N1(), 2, 2, .1);
  public static final Matrix<N3, N1> visionMatrix = MatBuilder.fill(Nat.N3(), Nat.N1(), 5, 5, 100);

  public static final Map<FieldPose, Translation2d> fieldPoseOffsets =
      // vision can't handle sideloading due to camera placement
      Map.ofEntries(
          // processor
          entry(
              new FieldPose(Alliance.Blue, FieldElement.P, Offset.MID),
              new Translation2d(0.0, 0.0)),

          // frontload top station AND bottom HP station
          entry(
              new FieldPose(Alliance.Blue, FieldElement.HT, Offset.MID),
              new Translation2d(0.0, 0.0)),
          entry(
              new FieldPose(Alliance.Blue, FieldElement.HT, Offset.LEFT),
              new Translation2d(0.0, 0.0)),
          entry(
              new FieldPose(Alliance.Blue, FieldElement.HT, Offset.RIGHT),
              new Translation2d(0.0, 0.0)),

          // Reef offsets (THESE SHOULD ALL BE THE SAME, THEY ARE JUST HERE FOR CONSISTENCY )
          entry(
              new FieldPose(Alliance.Blue, FieldElement.RL, Offset.MID),
              new Translation2d(0.0, 0.0)),
          entry(
              new FieldPose(Alliance.Blue, FieldElement.RL, Offset.LEFT),
              new Translation2d(0.0, 0.0)),
          entry(
              new FieldPose(Alliance.Blue, FieldElement.RL, Offset.RIGHT),
              new Translation2d(0.0, 0.0))

          // TODO: Auto field flipping for blue side (or red side ig)
          );

  public static final double maxTimeUntilFallbackToOdometry = 1.0;
  public static final Transform3d CenterCamPose =
      new Transform3d(
          new Translation3d(
              Units.inchesToMeters(9.325), // X
              Units.inchesToMeters(0.3505), // Y
              Units.inchesToMeters(5.85)), // Z
          new Rotation3d(0, 0, 0));
  public static final Transform3d CoralCamPose =
      new Transform3d(
          new Translation3d(
              Units.inchesToMeters(7.262), // X
              Units.inchesToMeters(10.201), // Y
              Units.inchesToMeters(6.638)), // Z
          new Rotation3d(0, 0, 0));
}
