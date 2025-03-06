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

  // TODO figure out offsets
  public static final Map<String, Translation2d> fieldPoseToCameraAngleOffset =
      // vision can't handle sideloading due to camera placement

      // DOUBLES ARE [YAW, PITCH]
      Map.ofEntries(
          // processor
          entry(
              new FieldPose(Alliance.Blue, FieldElement.P, Offset.MID).toString(),
              new Translation2d(0.0, 0.0)),
          // frontload top station AND bottom HP station
          entry(
              new FieldPose(Alliance.Blue, FieldElement.HT, Offset.MID).toString(),
              new Translation2d(0.0, 0.0)),
          entry(
              new FieldPose(Alliance.Blue, FieldElement.HT, Offset.LEFT).toString(),
              new Translation2d(0.0, 0.0)),
          entry(
              new FieldPose(Alliance.Blue, FieldElement.HT, Offset.RIGHT).toString(),
              new Translation2d(0.0, 0.0)),

          // Reef offsets (THESE SHOULD ALL BE THE SAME, THEY ARE JUST HERE FOR CONSISTENCY )
          entry(
              new FieldPose(Alliance.Blue, FieldElement.A, Offset.MID).toString(),
              new Translation2d(0.0, 0.0)),
          entry(
              new FieldPose(Alliance.Blue, FieldElement.A, Offset.LEFT).toString(),
              //   new Translation2d(11.87, -4.75)),
              new Translation2d(0.24, -.1)),
          entry(
              new FieldPose(Alliance.Blue, FieldElement.A, Offset.RIGHT).toString(),
              new Translation2d(-8, -4))

          // TODO: Auto field flipping for blue side (or red side ig)
          );

  public static final Map<Integer, FieldElement> tagIDToFieldElement =
      Map.ofEntries(
          // blue
          entry(18, FieldElement.A),
          entry(17, FieldElement.F),
          entry(22, FieldElement.E),
          entry(21, FieldElement.D),
          entry(20, FieldElement.C),
          entry(19, FieldElement.B),
          entry(13, FieldElement.HT),
          entry(12, FieldElement.HB),
          entry(16, FieldElement.P),
          entry(14, FieldElement.BT),
          entry(15, FieldElement.BB),
          // red
          entry(7, FieldElement.A),
          entry(8, FieldElement.F),
          entry(9, FieldElement.E),
          entry(10, FieldElement.D),
          entry(11, FieldElement.C),
          entry(6, FieldElement.B),
          entry(1, FieldElement.HT),
          entry(2, FieldElement.HB),
          entry(3, FieldElement.P),
          entry(5, FieldElement.BT),
          entry(4, FieldElement.BB));

  public static final double maxTimeUntilFallbackToOdometry = 1.0;
  public static final Transform3d LeftFacingCameraPose =
      new Transform3d(
          new Translation3d(
              Units.inchesToMeters(9.398), // X 9.325
              Units.inchesToMeters(7.551), // Y .3505
              Units.inchesToMeters(8.25)), // Z 5.85
          new Rotation3d(
              Units.degreesToRadians(0),
              Units.degreesToRadians(-20),
              Units.degreesToRadians(30))); // 0 0 0
  //   new Transform3d(
  //       new Translation3d(
  //           Units.inchesToMeters(10.382), // X 9.325
  //           Units.inchesToMeters(-11.941), // Y .3505
  //           Units.inchesToMeters(8.419)), // Z 5.85
  //       new Rotation3d(
  //           Units.degreesToRadians(0),
  //           Units.degreesToRadians(-20),
  //           Units.degreesToRadians(30))); // 0 0 0
  public static final Transform3d RightFacingCameraPose =
      new Transform3d(
          new Translation3d(
              Units.inchesToMeters(7.693), // X 7.262
              Units.inchesToMeters(10.815), // Y 10.201
              Units.inchesToMeters(8.251)), // Z 6.638
          new Rotation3d(
              Units.degreesToRadians(0),
              Units.degreesToRadians(-20),
              Units.degreesToRadians(-20))); // 0 0 0
  public static final Transform3d HPCameraPose =
      new Transform3d(
          new Translation3d(
              Units.inchesToMeters(0), Units.inchesToMeters(-6.033), Units.inchesToMeters(40.25)),
          new Rotation3d(
              Units.degreesToRadians(0), Units.degreesToRadians(0), Units.degreesToRadians(125)));
  public static final Transform3d CageDetectCameraPose =
      new Transform3d(
          new Translation3d(
              Units.inchesToMeters(.863),
              Units.inchesToMeters(-7.387),
              Units.inchesToMeters(34.334)),
          new Rotation3d(
              Units.degreesToRadians(0), Units.degreesToRadians(160), Units.degreesToRadians(160)));
  public static final Transform3d BranchCameraPose =
      new Transform3d(
          new Translation3d(
              Units.inchesToMeters(0.0), Units.inchesToMeters(0.0), Units.inchesToMeters(0.0)),
          new Rotation3d(
              Units.degreesToRadians(0), Units.degreesToRadians(0), Units.degreesToRadians(0)));
  public static final double finalYawSetpointLeft = 13.8;
  public static final double finalPitchSetpointLeft = -3.37;
  public static final double finalYawSetpointRight = -7.97;
  public static final double finalPitchSetpointRight = -7.88;

  // color branch align
  // TODO tune all
  public static final double heightThreshold = 0.0;
  public static final double widthThreshold = 0.0;
  public static final double widthSetpoint = 0.0;
  public static final double xSetpoint = 0.0;
  public static final double widthTolerance = 0.0;
  public static final double xTolerance = 0.0;

  public static final double heightWidthRatioThreshold = 0.0;
}
