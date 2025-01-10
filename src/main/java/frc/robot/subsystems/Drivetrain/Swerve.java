package frc.robot.subsystems.Drivetrain;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;

import choreo.trajectory.SwerveSample;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
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
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.dashboard.TunableNumber;
import frc.robot.Constants.DrivetrainConstants;

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

  @Logged(name = "mod/Front Left", importance = Importance.CRITICAL)
  private TalonFXSwerveModule m_FrontLeft;

  @Logged(name = "mod/Front Right", importance = Importance.CRITICAL)
  private TalonFXSwerveModule m_FrontRight;

  @Logged(name = "mod/Back Left", importance = Importance.CRITICAL)
  private TalonFXSwerveModule m_BackLeft;

  @Logged(name = "mod/Back Right", importance = Importance.CRITICAL)
  private TalonFXSwerveModule m_BackRight;

  @Logged(name = "Drive Characterization Current", importance = Importance.DEBUG)
  private double CharacterizeCurrent = 0;

  // controllers for autos
  private final PIDController xController;
  private final PIDController yController;
  private final PIDController rController;


  // fuse camera pose into odometry
  @Logged(name = "PDH", importance = Importance.DEBUG)
  private final PowerDistribution m_PDH;

  // gryo
  private Pigeon2 m_gyro;

  /** define swerve modules, Gyro, odometry */
  public Swerve() {

    m_PDH = new PowerDistribution(1, ModuleType.kRev);

    // initalize objects in constructor so that they dont get initialized when the
    // subsystem is not initialized
    m_gyro = new Pigeon2(DrivetrainConstants.kPigeonID, DrivetrainConstants.driveBusName);
    m_gyro.getConfigurator().apply(new Pigeon2Configuration());
    m_gyro.setYaw(0);

    zeroGyro();

    m_FrontLeft = new TalonFXSwerveModule(0, DrivetrainConstants.FrontLeft.constants);
    m_FrontRight = new TalonFXSwerveModule(1, DrivetrainConstants.FrontRight.constants);
    m_BackLeft = new TalonFXSwerveModule(2, DrivetrainConstants.BackLeft.constants);
    m_BackRight = new TalonFXSwerveModule(3, DrivetrainConstants.BackRight.constants);

    m_SwerveModules =
        new TalonFXSwerveModule[] {m_FrontLeft, m_FrontRight, m_BackLeft, m_BackRight};

    Pose2d initialPose = new Pose2d(0, 0, new Rotation2d(0));

    xController = new PIDController(2.65, 0, 0);
    yController = new PIDController(3.9, 0, 0);
    rController = new PIDController(3.05, 0, 0);

    swerveOdometry =
        new SwerveDrivePoseEstimator(
            DrivetrainConstants.swerveKinematics,
            getRotation2D(),
            getModulePositions(),
            initialPose);
  }

  /**
   * @param translation - X (Meters per second, Forwards/Backwards) and Y (Meters Per Second,
   *     Left/Right)
   * @param rotation - Yaw/angle of the robot (Counter Clockwise is positive)
   * @param openLoop - Use feedback and PID (if false)
   */
  public void drive(
      Translation2d translation, double rotation, boolean isOpenLoop, boolean fieldRelative) {
    SwerveModuleState[] swerveModuleStates =
        DrivetrainConstants.swerveKinematics.toSwerveModuleStates(
            fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(
                    translation.getX(), translation.getY(), rotation, getRotation2D())
                : new ChassisSpeeds(translation.getX(), translation.getY(), rotation));

    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, DrivetrainConstants.maxSpeed);

    for (TalonFXSwerveModule mod : m_SwerveModules) {
      mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop, false);
    }
  }
  /**
   * intermediary function to convert between chassis speeds and swerve module states
   * @param speeds
   */
  public void setChassisSpeeds(ChassisSpeeds speeds) {
    speeds.omegaRadiansPerSecond = speeds.omegaRadiansPerSecond;
    speeds = setpointSpeeds;
    var swerveModuleStates =
        DrivetrainConstants.swerveKinematics.toSwerveModuleStates(speeds, new Translation2d(0, 0));
    setModuleStates(swerveModuleStates, false);
  }
  /**
   * getter for chassis speeds
   * @return robot relative speeds
   */
  public ChassisSpeeds getChassisSpeeds() {
    return DrivetrainConstants.swerveKinematics.toChassisSpeeds(getModuleStates());
  }

  /**
   * follows an autonomous path provided by choreo
   */
  public void followSwerveSample(SwerveSample sample) {
    // TODO: some fancy optimization stuff
    ChassisSpeeds speeds =
        ChassisSpeeds.fromFieldRelativeSpeeds(
            new ChassisSpeeds(
                xController.calculate(getPose().getX(), sample.x) + sample.vx,
                yController.calculate(getPose().getY(), sample.y) + sample.vy,
                rController.calculate(getPose().getRotation().getRadians(), sample.heading)
                    + sample.omega),
            getPose().getRotation());
    this.setChassisSpeeds(speeds);
  }

  /**
   * Set the module states (used in autos)
   *
   * @param desiredStates The desired module state to set the wheels
   */
  public void setModuleStates(SwerveModuleState[] desiredStates, boolean steerWhenStationary) {
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DrivetrainConstants.maxSpeed);
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
   * @param current
   */
  public void characterizeDrive(Current current) {
    CharacterizeCurrent = current.in(Amps);
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
   * returns the velocity and angle of all swerve modules
   *
   * @return the state of all swerve modules
   */
  @Logged(name = "Module States", importance = Importance.CRITICAL)
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
  @Logged(name = "Module Setpoints", importance = Importance.CRITICAL)
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
  @Logged(name = "Module Positions", importance = Importance.INFO)
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

  /**
   * gives a breaking heading (360->0 degrees for example) Takes into account a gyro invert
   *
   * @return rotation2d returned by the gyro
   */
  @Logged(name = "Gyro Angle", importance = Importance.INFO)
  public Rotation2d getRotation2D() {
    return Rotation2d.fromDegrees(m_gyro.getYaw().getValue().in(Degrees));
  }

  @Override
  public void periodic() {
    swerveOdometry.update(getRotation2D(), getModulePositions());
  }
}
