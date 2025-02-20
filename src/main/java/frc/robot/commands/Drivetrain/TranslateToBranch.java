package frc.robot.commands.Drivetrain;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Importance;
import edu.wpi.first.epilogue.Logged.Strategy;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.dashboard.TunableNumber;
import frc.robot.subsystems.Drivetrain.Swerve;
import frc.robot.subsystems.Vision.AprilTagCamera;
import java.util.Collections;
import java.util.Comparator;
import java.util.HashMap;
import java.util.Map;
import java.util.OptionalDouble;
import java.util.Set;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;


@Logged(name = "TranslateToBranch", strategy = Strategy.OPT_IN)
public class TranslateToBranch extends Command {

  private PIDController xController;
  private PIDController yController;

  private final AprilTagCamera m_AprilTagCamera;
  private final Swerve m_Swerve;
  private boolean alignLeft;

  private final double targetPitchLeft = -5;
  private final double targetPitchRight = 0.5;
  private final double targetYawLeft = 13;
  private final double targetYawRight = 0;

  private OptionalDouble currentPitch;
  private OptionalDouble currentYaw;

  @Logged(name = "TranslateToBranch/XCommand", importance = Importance.CRITICAL)
  private double xCommand;
  @Logged(name = "TranslateToBranch/YCommand", importance = Importance.CRITICAL)
  private double yCommand;

  private TunableNumber[] xPID = {
    new TunableNumber("TranslateToBranch/xP", 0.005),
    new TunableNumber("TranslateToBranch/xI", 0),
    new TunableNumber("TranslateToBranch/xD", 0.0001)
  };

  private TunableNumber[] yPID = {
    new TunableNumber("TranslateToBranch/yP", 0.001),
    new TunableNumber("TranslateToBranch/yI", 0),
    new TunableNumber("TranslateToBranch/yD", 0.0001)
  };

  private final Map<Double, Set<Integer>> headingToTagIDs;
  {
    headingToTagIDs = new HashMap<>();
    headingToTagIDs.put(Math.toRadians(0), Set.of(18, 7));
    headingToTagIDs.put(Math.toRadians(60), Set.of(8, 17));
    headingToTagIDs.put(Math.toRadians(120), Set.of(9, 22));
    headingToTagIDs.put(Math.toRadians(180), Set.of(10, 21));
    headingToTagIDs.put(Math.toRadians(240), Set.of(11, 20));
    headingToTagIDs.put(Math.toRadians(300), Set.of(22, 6));
  }

  private Set<Integer> correspondingTagIDs;
  private int tagIDToFocus;








  public TranslateToBranch(Swerve swerve, AprilTagCamera camera, boolean alignLeft) {
    m_Swerve = swerve;
    m_AprilTagCamera = camera;
    this.alignLeft = alignLeft;

    xController = new PIDController(xPID[0].getNumber(), xPID[1].getNumber(), xPID[2].getNumber());
    yController = new PIDController(yPID[0].getNumber(), yPID[1].getNumber(), yPID[2].getNumber());

    addRequirements(m_Swerve);
  }



  @Override
  public void initialize() {
    if (alignLeft) {
      xController.setSetpoint(targetPitchLeft);
      yController.setSetpoint(targetYawLeft);
    } else {
      xController.setSetpoint(targetPitchRight);
      yController.setSetpoint(targetPitchLeft);
    }
    xController.setTolerance(5);
    yController.setTolerance(5);

    double heading = m_Swerve.getPose().getRotation().getRadians();
    double nearestHeading = Collections.min(headingToTagIDs.keySet(), 
    Comparator.comparingDouble(h -> Math.abs(h - heading)));
    correspondingTagIDs = headingToTagIDs.get(nearestHeading); // get the set of 2 tagIDs that correspond to the nearest heading

    // Determine the tagIDToFocus based on the alliance
    Alliance alliance = DriverStation.getAlliance().get();
    if (alliance == Alliance.Red) {
      tagIDToFocus = correspondingTagIDs.stream()
        .filter(id -> id >= 6 && id <= 11)
        .findFirst()
        .orElseThrow(() -> new IllegalStateException("No valid tag ID found for Red alliance"));
    } else if (alliance == Alliance.Blue) {
      tagIDToFocus = correspondingTagIDs.stream()
        .filter(id -> id >= 17 && id <= 22)
        .findFirst()
        .orElseThrow(() -> new IllegalStateException("No valid tag ID found for Blue alliance"));
    } else {
      throw new IllegalStateException("Unknown alliance: " + alliance);
    }

    // tagIDToFocus is the tagID which relates to the nearest heading and the alliance
  }



  @Override
  public void execute() {
    xController.setPID(xPID[0].getNumber(), xPID[1].getNumber(), xPID[2].getNumber());
    yController.setPID(yPID[0].getNumber(), yPID[1].getNumber(), yPID[2].getNumber());
    
    
    if (m_AprilTagCamera.getTarget(tagIDToFocus).isPresent()) { // ensure tag to focus in view
      currentPitch = m_AprilTagCamera.getY(tagIDToFocus); // Y is Pitch
      currentYaw = m_AprilTagCamera.getX(tagIDToFocus); // X is Yaw
      if (alignLeft) {
        xCommand = xController.calculate(currentPitch.getAsDouble(), targetPitchLeft);
        yCommand = yController.calculate(currentYaw.getAsDouble(), targetYawLeft);
      } else {
        xCommand = xController.calculate(currentPitch.getAsDouble(), targetPitchRight);
        yCommand = yController.calculate(currentYaw.getAsDouble(), targetYawRight);
      }

      // m_Swerve.drive(new Translation2d(xCommand, yCommand), 0); // needs to be robot relative
      m_Swerve.setRobotRelative(new ChassisSpeeds(xCommand, yCommand, 0));
    }
  }



  @Override
  public void end(boolean interrupted) {}



  @Override
  public boolean isFinished() {
    // return false;
    return (xController.atSetpoint() && yController.atSetpoint());
  }
}
