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
import java.util.HashSet;
import java.util.OptionalDouble;
import java.util.OptionalInt;
import java.util.Set;


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
  private OptionalInt primaryTagID;

  @Logged(name = "TranslateToBranch/XCommand", importance = Importance.CRITICAL)
  private double xCommand;
  @Logged(name = "TranslateToBranch/YCommand", importance = Importance.CRITICAL)
  private double yCommand;

  private TunableNumber[] xPID = {
    new TunableNumber("TranslateToBranch/xP", 0.01),
    new TunableNumber("TranslateToBranch/xI", 0),
    new TunableNumber("TranslateToBranch/xD", 0)
  };

  private TunableNumber[] yPID = {
    new TunableNumber("TranslateToBranch/yP", 0.05),
    new TunableNumber("TranslateToBranch/yI", 0),
    new TunableNumber("TranslateToBranch/yD", 0)
  };

  private final Set<Integer> reefTagIDs;

  public TranslateToBranch(Swerve swerve, AprilTagCamera camera, boolean alignLeft) {
    m_Swerve = swerve;
    m_AprilTagCamera = camera;
    this.alignLeft = alignLeft;

    xController = new PIDController(xPID[0].getNumber(), xPID[1].getNumber(), xPID[2].getNumber());
    yController = new PIDController(yPID[0].getNumber(), yPID[1].getNumber(), yPID[2].getNumber());

    reefTagIDs = new HashSet<>();
    for (int i = 6; i <= 11; i++) {
      reefTagIDs.add(i);
    }
    for (int i = 17; i <= 22; i++) {
      reefTagIDs.add(i);
    }

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
    xController.setTolerance(6);
    yController.setTolerance(6);
  }

  @Override
  public void execute() {
    xController.setPID(xPID[0].getNumber(), xPID[1].getNumber(), xPID[2].getNumber());
    yController.setPID(yPID[0].getNumber(), yPID[1].getNumber(), yPID[2].getNumber());
    
    
    primaryTagID = OptionalInt.empty();


    primaryTagID = m_AprilTagCamera.getPrimaryTagID();
    if (m_AprilTagCamera.hasTarget() && reefTagIDs.contains(primaryTagID.getAsInt())) {
      currentPitch = m_AprilTagCamera.getY(primaryTagID.getAsInt()); // Y is Pitch
      currentYaw = m_AprilTagCamera.getX(primaryTagID.getAsInt()); // X is Yaw
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
