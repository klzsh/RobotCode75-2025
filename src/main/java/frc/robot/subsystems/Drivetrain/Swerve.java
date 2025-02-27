package frc.robot.subsystems.Drivetrain;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Rotation;
import static frc.robot.Constants.DrivetrainConstants.*;
import static frc.robot.Constants.DrivetrainConstants.MotorConfigs.*;
import static frc.robot.Constants.VisionConstants.moduleMatrix;
import static frc.robot.Constants.VisionConstants.visionMatrix;

import choreo.trajectory.SwerveSample;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Importance;
import edu.wpi.first.epilogue.Logged.Strategy;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.dashboard.TunableNumber;
import frc.robot.Constants.DrivetrainConstants.BackLeft;
import frc.robot.Constants.DrivetrainConstants.BackRight;
import frc.robot.Constants.DrivetrainConstants.FrontLeft;
import frc.robot.Constants.DrivetrainConstants.FrontRight;
import frc.robot.subsystems.Vision.AprilTagCamera;

/*
 * OVERVIEW OF CHASSIS
 * implements kinematics and odometry, as well as drive methods to interface with the swerve module
 * each module is defined here, as well as the odometry object and the gyro
 * has methods to get and set individual module positions (for autos or X-Stance) and for teleop driving
 * also contains most of the logging for the chassis (voltage/current, speed, etc)
 */
@Logged(strategy = Strategy.OPT_IN, name = "Drivetrain")
public class Swerve extends SubsystemBase {
  private SwerveDrivePoseEstimator swerveOdometry;

  public TalonFXSwerveModule[] m_SwerveModules;

  @Logged(name = "Chassis Speeds", importance = Importance.DEBUG)
  private ChassisSpeeds setpointSpeeds = new ChassisSpeeds();

  // for logging purposes. they are passed through to the m_SwerveModules array in
  // the constructor
  // TODO: tunable numebr for PIDs + current limits
  // @Logged(name = "mod/Front Left", importance = Importance.CRITICAL)
  private TalonFXSwerveModule m_FrontLeft;

  // @Logged(name = "mod/Front Right", importance = Importance.CRITICAL)
  private TalonFXSwerveModule m_FrontRight;

  // @Logged(name = "mod/Back Left", importance = Importance.CRITICAL)
  private TalonFXSwerveModule m_BackLeft;

  // @Logged(name = "mod/Back Right", importance = Importance.CRITICAL)
  private TalonFXSwerveModule m_BackRight;

  private final TunableNumber driveKP;
  private final TunableNumber driveKI;
  private final TunableNumber driveKD;
  private final TunableNumber driveKS;

  private final Slot0Configs drivePIDS;

  // controllers for autos
  private final PIDController xController;
  private final PIDController yController;
  private final PIDController rController;

  private final AprilTagCamera m_RightFacingCamera;
  private final AprilTagCamera m_LeftFacingCamera;
  private final AprilTagCamera m_HPCamera;

  // do this later
  private final TunableNumber xKP;
  private final TunableNumber xKI;
  private final TunableNumber xKD;

  private final TunableNumber yKP;
  private final TunableNumber yKI;
  private final TunableNumber yKD;

  private final TunableNumber rotationKP;
  private final TunableNumber rotationKI;
  private final TunableNumber rotationKD;

  private final TunableNumber maxAutosSpeed;

  private double lastUpdatedTime = 0;
  private boolean m_FieldRelative = true;

  @Logged(name = "Swerve/Sample Pose", importance = Importance.CRITICAL)
  private Pose2d sample;

  // gyro
  private Pigeon2 m_gyro;

  /** define swerve modules, Gyro, odometry */
  public Swerve(
      AprilTagCamera leftFacingCamera, AprilTagCamera rightFacingCamera, AprilTagCamera HPCamera) {

    sample = null;
    m_LeftFacingCamera = leftFacingCamera;
    m_RightFacingCamera = rightFacingCamera;
    m_HPCamera = HPCamera;

    // initalize objects in constructor so that they dont get initialized when the
    // subsystem is not initialized
    m_gyro = new Pigeon2(kPigeonID, driveBusName);
    m_gyro.getConfigurator().apply(new Pigeon2Configuration());
    m_gyro.setYaw(0);

    zeroGyro();

    m_FrontLeft = new TalonFXSwerveModule(0, FrontLeft.constants);
    m_FrontRight = new TalonFXSwerveModule(1, FrontRight.constants);
    m_BackLeft = new TalonFXSwerveModule(2, BackLeft.constants);
    m_BackRight = new TalonFXSwerveModule(3, BackRight.constants);

    m_SwerveModules =
        new TalonFXSwerveModule[] {m_FrontLeft, m_FrontRight, m_BackLeft, m_BackRight};

    Pose2d initialPose = new Pose2d(0, 0, new Rotation2d(0));
    // values from last year
    xController = new PIDController(2.65, 0, 0);
    yController = new PIDController(3.9, 0, 0);
    rController = new PIDController(3.05, 0, 0);
    rController.enableContinuousInput(-Math.PI, Math.PI);
    swerveOdometry =
        new SwerveDrivePoseEstimator(
            swerveKinematics,
            getRotation2D(),
            getModulePositions(),
            initialPose,
            moduleMatrix,
            visionMatrix);
    // init tunable numbers
    driveKP = new TunableNumber("Swerve/DriveMotor/kP", driveTorqueKP);
    driveKI = new TunableNumber("Swerve/DriveMotor/kI", driveTorqueKI);
    driveKD = new TunableNumber("Swerve/DriveMotor/kD", driveTorqueKD);
    driveKS = new TunableNumber("Swerve/DriveMotor/kS", driveTorqueKS);

    xKP = new TunableNumber("Autos/X-KP", 2.65);
    xKI = new TunableNumber("Autos/X-KI", 0);
    xKD = new TunableNumber("Autos/X-KD", 0);

    yKP = new TunableNumber("Autos/Y-KP", 3.9);
    yKI = new TunableNumber("Autos/Y-KI", 0);
    yKD = new TunableNumber("Autos/Y-KD", 0);
    rotationKP = new TunableNumber("Autos/Rotation-KP", 3.05);
    rotationKI = new TunableNumber("Autos/Rotation-KI", 0);
    rotationKD = new TunableNumber("Autos/Rotation-KD", 0);

    maxAutosSpeed = new TunableNumber("Autos/maxSpeed", 1);

    drivePIDS =
        new Slot0Configs()
            .withKP(driveTorqueKP)
            .withKI(0)
            .withKD(driveTorqueKD)
            .withKS(driveTorqueKS);
    // setPoseByVision(m_LeftFacingCamera);
  }

  /**
   * @param translation - X (Meters per second, Forwards/Backwards) and Y (Meters Per Second,
   *     Left/Right)
   * @param rotation - Yaw/angle of the robot (Counter Clockwise is positive)
   * @param openLoop - Use feedback and PID (if false)
   */
  public void drive(Translation2d translation, double rotation) {
    SwerveModuleState[] swerveModuleStates =
        swerveKinematics.toSwerveModuleStates(
            m_FieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(
                    translation.getX(), translation.getY(), rotation, getRotation2D())
                : new ChassisSpeeds(translation.getX(), translation.getY(), rotation));

    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, maxSpeed.in(MetersPerSecond));

    for (TalonFXSwerveModule mod : m_SwerveModules) {
      mod.setDesiredState(swerveModuleStates[mod.moduleNumber], false, false);
    }
  }

  public void toggleRobotRelative() {
    m_FieldRelative = false;
  }

  public void toggleFieldRelative() {
    m_FieldRelative = true;
  }

  /**
   * intermediary function to convert between chassis speeds and swerve module states
   *
   * @param speeds
   */
  public void setChassisSpeeds(ChassisSpeeds speeds) {
    setpointSpeeds = speeds;
    var swerveModuleStates = swerveKinematics.toSwerveModuleStates(speeds);
    setModuleStates(swerveModuleStates, false);
  }

  public void setFieldRelative(ChassisSpeeds speeds) {
    setpointSpeeds = speeds;
    var states =
        swerveKinematics.toSwerveModuleStates(
            ChassisSpeeds.fromFieldRelativeSpeeds(speeds, getRotation2D()));
    setModuleStates(states, false);
  }

  public void setRobotRelative(ChassisSpeeds speeds) {
    setpointSpeeds = speeds;
    var states =
        swerveKinematics.toSwerveModuleStates(
            ChassisSpeeds.fromRobotRelativeSpeeds(speeds, getRotation2D()));
    setModuleStates(states, false);
  }

  /**
   * getter for chassis speeds
   *
   * @return robot relative speeds
   */
  public ChassisSpeeds getChassisSpeeds() {
    return swerveKinematics.toChassisSpeeds(getModuleStates());
  }

  /** follows an autonomous path provided by choreo */
  public void followSwerveSample(SwerveSample sample) {
    this.sample = sample.getPose();
    Pose2d pose = getPose();
    ChassisSpeeds speeds =
        new ChassisSpeeds(
            sample.vx + xController.calculate(pose.getX(), sample.x),
            sample.vy + yController.calculate(pose.getY(), sample.y),
            sample.omega + rController.calculate(pose.getRotation().getRadians(), sample.heading));
    this.setFieldRelative(speeds);
  }

  /**
   * Set the module states (used in autos)
   *
   * @param desiredStates The desired module state to set the wheels
   */
  public void setModuleStates(SwerveModuleState[] desiredStates, boolean steerWhenStationary) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, 3.5); // 4.7 // TODO: change this back to controller constants
    for (TalonFXSwerveModule mod : m_SwerveModules) {
      mod.setDesiredState(desiredStates[mod.moduleNumber], false, steerWhenStationary);
    }
  }

  /** stops the swerve modules for autonomous */
  public void stopModules() {
    for (TalonFXSwerveModule mod : m_SwerveModules) {
      mod.setDesiredState(new SwerveModuleState(0, mod.getSetpoint().angle), true, false);
    }
  }

  /**
   * Used to find the kS value of the swerve module drive motors. Essentially a feedforward
   *
   * @param current
   */
  public void characterizeDrive(Current current) {
    for (TalonFXSwerveModule mod : m_SwerveModules) {
      mod.characterizeDrive(current);
    }
  }

  /**
   * @return the estimated position of the robot
   */
  @Logged(name = "Robot Pose", importance = Importance.CRITICAL)
  public Pose2d getPose() {
    return swerveOdometry.getEstimatedPosition();
  }

  /**
   * resets the robot pose
   *
   * @param pose the desired pose to reset the robot to
   */
  public void setPose(Pose2d pose) {
    swerveOdometry.resetPosition(getRotation2D(), getModulePositions(), pose);
  }

  /**
   * sets the pose by the vision odometry if the deviation between vision and module odometry is too
   * much
   *
   * @param camera the camera to set the odoemtry with
   */
  public void setPoseByVision(AprilTagCamera camera) {
    if (camera.getEstimatedPose() != null) {
      swerveOdometry.resetPosition(
          getRotation2D(),
          getModulePositions(),
          camera.getEstimatedPose().estimatedPose.toPose2d());
    }
  }

  public void updatePoseByVision(AprilTagCamera camera) {
    double timestamp = Timer.getFPGATimestamp();
    // if (timestamp - lastUpdatedTime < 0.005) {
    //   return;
    // }

    lastUpdatedTime = timestamp;
    if (camera.getEstimatedPose() != null) {
      swerveOdometry.addVisionMeasurement(
          camera.getEstimatedPose().estimatedPose.toPose2d(),
          camera.getEstimatedPose().timestampSeconds);
    }
  }

  /**
   * returns the velocity and angle of all swerve modules
   *
   * @return the state of all swerve modules
   */
  // @Logged(name = "Module States", importance = Importance.CRITICAL)
  public SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (TalonFXSwerveModule mod : m_SwerveModules) {
      states[mod.moduleNumber] = mod.getState();
    }

    return states;
  }

  /**
   * used for swerve drive logging
   *
   * @return the setpoint a module has been set to
   */
  // @Logged(name = "Module Setpoints", importance = Importance.CRITICAL)
  public SwerveModuleState[] getModuleSetpoints() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (TalonFXSwerveModule mod : m_SwerveModules) {
      states[mod.moduleNumber] = mod.getSetpoint();
    }
    return states;
  }

  /**
   * returns the meters travelled and angle of the swerve module
   *
   * @return position of all swerve modules
   */
  // @Logged(name = "Module Positions", importance = Importance.INFO)
  public SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] positions = new SwerveModulePosition[4];
    for (TalonFXSwerveModule mod : m_SwerveModules) {
      positions[mod.moduleNumber] = mod.getPosition();
    }
    return positions;
  }

  /** resets the Pigeon Gryo (sets angle to 0) */
  public void zeroGyro() {
    m_gyro.setYaw(0);
  }

  public void zeroGyro(Rotation2d startVal) {
    m_gyro.setYaw(startVal.getDegrees());
  }

  /** gives a breaking heading (360->0 degrees for example) Takes into account a gyro invert */
  public Rotation2d getRotation2D() {
    return Rotation2d.fromDegrees(m_gyro.getYaw(true).getValue().in(Degrees));
  }

  @Logged(name = "Gyro Angle Degrees", importance = Importance.CRITICAL)
  public double getRotationDegrees() {
    return m_gyro.getYaw(true).getValue().in(Degrees);
  }

  // @Logged(name = "Pose to Drive", importance = Importance.CRITICAL)
  private Pose2d poseToDrive;

  @Override
  public void periodic() {
    // m_LeftFacingCamera.updateHeading(getRotation2D());
    // updatePoseByVision(m_LeftFacingCamera);
    // m_RightFacingCamera.updateHeading(getRotation2D());
    // updatePoseByVision(m_RightFacingCamera);
    // m_HPCamera.updateHeading(getRotation2D());
    // updatePoseByVision(m_HPCamera);

    swerveOdometry.update(getRotation2D(), getModulePositions());

    // set odometry to vision pose if it deviates by more than half a meter

    // if (m_LeftFacingCamera.getEstimatedPose() != null) {
    //   if (Math.abs(
    //               m_LeftFacingCamera.getEstimatedPose().estimatedPose.getX()
    //                   - swerveOdometry.getEstimatedPosition().getX())
    //           > .5
    //       || Math.abs(
    //               m_LeftFacingCamera.getEstimatedPose().estimatedPose.getY()
    //                   - swerveOdometry.getEstimatedPosition().getY())
    //           > .5) {
    //     setPoseByVision(m_LeftFacingCamera);
    //   }
    // }
    // if (m_RightFacingCamera.getEstimatedPose() != null) {
    //   if (Math.abs(
    //               m_RightFacingCamera.getEstimatedPose().estimatedPose.getX()
    //                   - swerveOdometry.getEstimatedPosition().getX())
    //           > .5
    //       || Math.abs(
    //               m_RightFacingCamera.getEstimatedPose().estimatedPose.getY()
    //                   - swerveOdometry.getEstimatedPosition().getY())
    //           > .5) {
    //     setPoseByVision(m_RightFacingCamera);
    //   }
    // }
    // if (m_HPCamera.getEstimatedPose() != null) {
    //   if (Math.abs(
    //               m_HPCamera.getEstimatedPose().estimatedPose.getX()
    //                   - swerveOdometry.getEstimatedPosition().getX())
    //           > .5
    //       || Math.abs(
    //               m_HPCamera.getEstimatedPose().estimatedPose.getY()
    //                   - swerveOdometry.getEstimatedPosition().getY())
    //           > .5) {
    //     setPoseByVision(m_HPCamera);
    //   }
    // }
    if (driveKP.getNumber() != drivePIDS.kP
        || driveKI.getNumber() != drivePIDS.kI
        || driveKD.getNumber() != drivePIDS.kD
        || driveKS.getNumber() != drivePIDS.kS) {

      drivePIDS.kP = driveKP.getNumber();
      drivePIDS.kI = driveKI.getNumber();
      drivePIDS.kD = driveKI.getNumber();
      drivePIDS.kS = driveKD.getNumber();

      for (TalonFXSwerveModule mod : m_SwerveModules) {
        mod.setDrivePIDS(drivePIDS);
      }
    }
    if (xKP.getNumber() != xController.getP()
        || xKI.getNumber() != xController.getI()
        || xKD.getNumber() != xController.getD()) {
      xController.setP(xKP.getNumber());
      xController.setI(xKI.getNumber());
      xController.setD(xKD.getNumber());
    }
    if (yKP.getNumber() != yController.getP()
        || yKI.getNumber() != yController.getI()
        || yKD.getNumber() != yController.getD()) {
      yController.setP(yKP.getNumber());
      yController.setI(yKI.getNumber());
      yController.setD(yKD.getNumber());
    }
    if (rotationKP.getNumber() != rController.getP()
        || rotationKI.getNumber() != rController.getI()
        || rotationKD.getNumber() != rController.getD()) {
      rController.setP(rotationKP.getNumber());
      rController.setI(rotationKI.getNumber());
      rController.setD(rotationKD.getNumber());
    }
  }
}
