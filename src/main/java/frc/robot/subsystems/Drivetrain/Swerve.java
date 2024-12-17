package frc.robot.subsystems.Drivetrain;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;

import choreo.trajectory.SwerveSample;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
// import com.pathplanner.lib.auto.AutoBuilder;
// import com.pathplanner.lib.config.ModuleConfig;
// import com.pathplanner.lib.config.PIDConstants;
// import com.pathplanner.lib.config.RobotConfig;
// import com.pathplanner.lib.controllers.PPHolonomicDriveController;
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
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
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

  @Logged(name = "Driver X", importance = Importance.DEBUG)
  public double translationX = 0;

  @Logged(name = "Driver Y", importance = Importance.DEBUG)
  public double translationY = 0;

  @Logged(name = "Chassis Speeds", importance = Importance.DEBUG)
  private ChassisSpeeds setpointSpeeds = new ChassisSpeeds();

  // for logging purposes. they are passed through to the m_SwerveModules array in
  // the constructor

  @Logged(name = "mod/Front Left")
  private TalonFXSwerveModule m_FrontLeft;

  @Logged(name = "mod/Front Right")
  private TalonFXSwerveModule m_FrontRight;

  @Logged(name = "mod/Back Left")
  private TalonFXSwerveModule m_BackLeft;

  @Logged(name = "mod/Back Right")
  private TalonFXSwerveModule m_BackRight;

  private final Compressor m_Compressor;

  @Logged private double CharacterizeCurrent = 0;

  // controllers for autos
  private final PIDController xController;
  private final PIDController yController;
  private final PIDController rController;

  private final Field2d m_Field;

  // private final RobotConfig config;

  // fuse camera pose into odometry
  @Logged(name = "PDH", importance = Importance.DEBUG)
  private final PowerDistribution m_PDH;

  // gryo
  private Pigeon2 m_gyro;

  /** define swerve modules, AHRS, odometry */
  public Swerve() {
    // m_Intake = intake;

    m_Field = new Field2d();
    m_Compressor = new Compressor(PneumaticsModuleType.REVPH);
    // m_Compressor.enableAnalog(110, 120);
    m_Compressor.disable();
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
    // ! Uncomment if we decide to use pathplanner
    // config =
    //     new RobotConfig(
    //         Pounds.of(135),
    //         KilogramSquareMeters.of(6),
    //         new ModuleConfig(
    //             Inches.of(2),
    //             MetersPerSecond.of(3.779),
    //             1.0,
    //             DCMotor.getKrakenX60Foc(1),
    //             Amps.of(80),
    //             1),
    //         Inches.of(22.5),
    //         Inches.of(22.5));

    // AutoBuilder.configure(
    //     this::getPose,
    //     this::setPose,
    //     this::getChassisSpeeds,
    //     (speeds) -> setChassisSpeeds(speeds),
    //     new PPHolonomicDriveController(new PIDConstants(5, 0, 0), new PIDConstants(5, 0, 0)),
    //     config,
    //     () -> {
    //       // Boolean supplier that controls when the path will be mirrored for the red
    //       // alliance
    //       // This will flip the path being followed to the red side of the field.
    //       // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

    //       var alliance = DriverStation.getAlliance();
    //       if (alliance.isPresent()) {
    //         return alliance.get() == DriverStation.Alliance.Red;
    //       }
    //       return false;
    //     },
    //     this);
  }

  /**
   * @param translation - X (Meters per second, Forwards/Backwards) and Y (Meters Per Second,
   *     Left/Right)
   * @param rotation - Yaw/angle of the robot (Counter Clockwise is positive)
   * @param openLoop - Use feedback and PID (if false)
   */
  public void drive(
      Translation2d translation, double rotation, boolean isOpenLoop, boolean fieldRelative) {
    // here the translational speed is correct
    translationX = translation.getX();
    translationY = translation.getY();

    SwerveModuleState[] swerveModuleStates =
        DrivetrainConstants.swerveKinematics.toSwerveModuleStates(
            fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(
                    translation.getX(), translation.getY(), rotation, getRotation2D())
                : new ChassisSpeeds(translation.getX(), translation.getY(), rotation));

    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, DrivetrainConstants.maxSpeed);

    for (TalonFXSwerveModule mod : m_SwerveModules) {
      mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
    }
  }

  public void setChassisSpeeds(ChassisSpeeds speeds) {
    speeds.omegaRadiansPerSecond = -speeds.omegaRadiansPerSecond;
    speeds = setpointSpeeds;
    var swerveModuleStates =
        DrivetrainConstants.swerveKinematics.toSwerveModuleStates(speeds, new Translation2d(0, 0));
    setModuleStates(swerveModuleStates);
  }

  public ChassisSpeeds getChassisSpeeds() {
    return DrivetrainConstants.swerveKinematics.toChassisSpeeds(getModuleStates());
  }

  public void followSwerveSample(Pose2d currentPose, SwerveSample sample) {
    // TODO: some fancy optimization stuff
    ChassisSpeeds speeds =
        ChassisSpeeds.fromFieldRelativeSpeeds(
            new ChassisSpeeds(
                xController.calculate(currentPose.getX(), sample.x) + sample.vx,
                yController.calculate(currentPose.getY(), sample.y) + sample.vy,
                rController.calculate(currentPose.getRotation().getRadians(), sample.heading)
                    + sample.omega),
            currentPose.getRotation());
    this.setChassisSpeeds(speeds);
  }

  /**
   * Set the module states (used in autos)
   *
   * @param desiredStates The desired module state to set the wheels
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DrivetrainConstants.maxSpeed);
    for (TalonFXSwerveModule mod : m_SwerveModules) {
      mod.setDesiredState(desiredStates[mod.moduleNumber], false); // ! idk why this is open loop
    }
  }

  /** stops the swerve modules for autonomous */
  public void stopModules() {
    for (TalonFXSwerveModule mod : m_SwerveModules) {
      mod.setDesiredState(new SwerveModuleState(0, mod.getSetpoint().angle), true);
    }
  }

  public void characterizeDrive(Current current) {
    CharacterizeCurrent = current.in(Amps);
    for (TalonFXSwerveModule mod : m_SwerveModules) {
      mod.characterizeDrive(current);
    }
  }

  /**
   * @return the estimated position of the robot
   */
  @Logged
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
  @Logged
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
  @Logged
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
  @Logged
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
  @Logged
  public Rotation2d getRotation2D() {
    return Rotation2d.fromDegrees(m_gyro.getYaw().getValue().in(Degrees));
  }

  /** resets the swerve module state to the angle offset in constants */
  @Override
  public void periodic() {
    swerveOdometry.update(getRotation2D(), getModulePositions());

    // logging
    for (TalonFXSwerveModule mod : m_SwerveModules) {
      SmartDashboard.putNumber(
          "Mod " + mod.moduleNumber + " CANcoder", mod.getCANCoder().getDegrees());
    }

    m_Field.setRobotPose(swerveOdometry.getEstimatedPosition());
    SmartDashboard.putData(m_Field);

    SmartDashboard.putNumber(
        "Heading", swerveOdometry.getEstimatedPosition().getRotation().getDegrees());
    SmartDashboard.putNumber("Robot x", swerveOdometry.getEstimatedPosition().getX());
    SmartDashboard.putNumber("Robot y", swerveOdometry.getEstimatedPosition().getY());

    SmartDashboard.putNumber("Pressure", m_Compressor.getPressure());
  }
}
