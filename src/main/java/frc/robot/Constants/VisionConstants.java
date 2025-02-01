package frc.robot.Constants;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

public class VisionConstants {
  public static final Matrix<N3, N1> moduleMatrix = MatBuilder.fill(Nat.N3(), Nat.N1(), 2, 2, .1);
  public static final Matrix<N3, N1> visionMatrix = MatBuilder.fill(Nat.N3(), Nat.N1(), 5, 5, 100);
}
