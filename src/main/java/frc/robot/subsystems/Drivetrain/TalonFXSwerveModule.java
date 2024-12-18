package frc.robot.subsystems.Drivetrain;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Importance;
import edu.wpi.first.epilogue.Logged.Strategy;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import frc.lib.dashboard.TuningTab;
import frc.lib.math.Conversions;
import frc.lib.util.CTREModuleState;
import frc.lib.util.SwerveModuleConstants;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.Constants.HardwareConstants;

/*
 * OVERVIEW OF A SWERVE MODULE
 * controls the direction and speed of a single swerve module wheel
 * uses two Kraken X60/ Falcon500 motors (just needs a TalonFX)
 * Uses a CANcoder for determining the angle offset of each wheel
 * Angle motor is set to position mode and uses a PID loop to servo to the correct position
 * Drive motor can either be direct (percent/duty cycle output) or velocity (rotations/sec) controlled
 * Configures each motor and CANcoder as well
 * can convert to swerve module state (speed (m/s) / angle (rotation2d))
 * and swerve module position (distance drive motor has gone (meters) / angle (rotation2d) )
 * mainly used as an abstraction layer
 */
@Logged(strategy = Strategy.OPT_IN)
public class TalonFXSwerveModule {
  public int moduleNumber;
  private Rotation2d angleOffset;
  private Rotation2d lastAngle;

  @Logged(name = "Angle Motor", importance = Importance.DEBUG)
  private TalonFX mAngleMotor;

  @Logged(name = "Drive Motor", importance = Importance.DEBUG)
  private TalonFX mDriveMotor;

  // sets the "forward" position of the wheel
  private CANcoder angleEncoder;
  // used for closed loop control

  private SwerveModuleState setpoint;

  /* drive motor control requests */
  // TODO: timesync control requests, in CTREConfigs + oneshot in here
  // https://v6.docs.ctr-electronics.com/en/latest/docs/api-reference/api-usage/status-signals.html
  // TODO: latency compensation
  // TODO: tune closed loop RampRate
  // see following link
  private final DutyCycleOut driveDutyCycle = new DutyCycleOut(0);
  // closed loop control
  private final VelocityTorqueCurrentFOC torqueDrivevelocity =
      new VelocityTorqueCurrentFOC(RotationsPerSecond.of(0));
  // SysID Characterization
  private final TorqueCurrentFOC torqueCharacterization = new TorqueCurrentFOC(Amps.of(0));

  /* angle motor control requests */
  private final PositionTorqueCurrentFOC torqueAngleCurrent =
      new PositionTorqueCurrentFOC(Degrees.of(0));

  public TalonFXSwerveModule(int moduleNumber, SwerveModuleConstants moduleConstants) {
    this.moduleNumber = moduleNumber;
    this.angleOffset = moduleConstants.angleOffset;

    /* Angle Encoder Config */
    angleEncoder = new CANcoder(moduleConstants.cancoderID, DrivetrainConstants.driveBusName);
    configAngleEncoder();

    /* Angle Motor Config */
    mAngleMotor = new TalonFX(moduleConstants.angleMotorID, DrivetrainConstants.driveBusName);
    TuningTab.addPIDTuner("Module " + moduleNumber + " Angle Motor", mAngleMotor);
    configAngleMotor();

    /* Drive Motor Config */
    mDriveMotor = new TalonFX(moduleConstants.driveMotorID, DrivetrainConstants.driveBusName);
    TuningTab.addPIDTuner("Module " + moduleNumber + " Angle Motor", mDriveMotor);
    configDriveMotor();

    lastAngle = getState().angle;
    setpoint = new SwerveModuleState();
  }

  /**
   * sets the module state for an individual module
   *
   * @param desiredState the velocity and angle of the module
   * @param isOpenLoop wether or not to use PIDF and motor feedback
   */
  public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop) {
    /*
     * This is a custom optimize function, since default WPILib optimize assumes
     * continuous controller which CTRE and Rev onboard is not
     */

    desiredState = CTREModuleState.optimize(desiredState, getState().angle);
    setpoint = desiredState;
    // cosine compensation
    desiredState.cosineScale(getAngle());
    setAngle(desiredState);
    // will either be percent output or velocity based on open loop
    setSpeed(desiredState, isOpenLoop);
  }

  /**
   * Sets the speed of the drive motor
   *
   * @param desiredState velocity and angle of module (only uses velocity component)
   * @param isOpenLoop weather or not to directly drive the motor or use PIDF and motor feedback
   */
  private void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop) {
    if (isOpenLoop) {
      driveDutyCycle.Output =
          desiredState.speedMetersPerSecond / DrivetrainConstants.maxSpeed.in(MetersPerSecond);
      mDriveMotor.setControl(driveDutyCycle.withEnableFOC(true));
    } else {
      mDriveMotor.setControl(
          torqueDrivevelocity.withVelocity(
              Conversions.MPSToTalon(
                  MetersPerSecond.of(desiredState.speedMetersPerSecond),
                  DrivetrainConstants.wheelCircumference,
                  DrivetrainConstants.driveGearRatio)));
    }
  }

  public void characterizeDrive(Current current) {
    mDriveMotor.setControl(torqueCharacterization.withOutput(current));
  }

  /**
   * Sets the angle of the module via position control
   *
   * @param desiredState the velocity and angle of the module (only uses angle)
   */
  private void setAngle(SwerveModuleState desiredState) {
    Rotation2d angle =
        (Math.abs(desiredState.speedMetersPerSecond)
                <= (DrivetrainConstants.maxSpeed.in(MetersPerSecond) * 0.01))
            ? lastAngle
            : desiredState
                .angle; // Prevent rotating module if speed is less then 1%. Prevents Jittering.

    mAngleMotor.setControl(
        torqueAngleCurrent.withPosition(
            Conversions.degreesToTalon(
                Degrees.of(angle.getDegrees()), DrivetrainConstants.angleGearRatio)));

    lastAngle = angle;
  }

  /**
   * @return the current angle of the angle motor
   */
  @Logged(name = "Module Angle", importance = Importance.INFO)
  public Rotation2d getAngle() {
    return Rotation2d.fromDegrees(
        Conversions.talonToDegrees(
                mAngleMotor.getPosition().getValue(), DrivetrainConstants.angleGearRatio)
            .in(Degrees));
  }

  /**
   * @return CANCoder angle
   */
  @Logged(name = "CANCoder angle", importance = Importance.DEBUG)
  public Rotation2d getCANCoder() {
    return Rotation2d.fromRotations(angleEncoder.getAbsolutePosition().getValue().in(Rotations));
  }

  /**
   * @return wait for thee CANCoder due to phoenix 6 status signals
   */
  private Rotation2d waitForCANcoder() {
    /* wait for up to 250ms for a new CANcoder position */
    return Rotation2d.fromRotations(
        angleEncoder.getAbsolutePosition().waitForUpdate(250).getValue().in(Rotations));
  }

  /** resets the module to the "forward" position */
  public void resetToAbsolute() {
    Angle absolutePosition =
        Conversions.degreesToTalon(
            Degrees.of(waitForCANcoder().getDegrees() - angleOffset.getDegrees()),
            DrivetrainConstants.angleGearRatio);
    mAngleMotor.setPosition(absolutePosition);
  }

  /** configures angle encoder based on ctreConfigs */
  private void configAngleEncoder() {
    angleEncoder.getConfigurator().apply(HardwareConstants.Swerve.getEncoderConfiguration());
  }

  /** configures angle motor based on ctreConfigs */
  private void configAngleMotor() {
    mAngleMotor.getConfigurator().apply(HardwareConstants.Swerve.getAngleConfiguration());
    resetToAbsolute();
  }

  /** configures drive motor based on ctreConfigs */
  private void configDriveMotor() {
    mDriveMotor.getConfigurator().apply(HardwareConstants.Swerve.getDriveConfiguration());
    mDriveMotor.getConfigurator().setPosition(0);
  }

  /**
   * @return the velocity and angle of the module
   */
  @Logged(name = "Module State", importance = Importance.CRITICAL)
  public SwerveModuleState getState() {
    return new SwerveModuleState(
        Conversions.talonToMPS(
            mDriveMotor.getVelocity().getValue(),
            DrivetrainConstants.wheelCircumference,
            DrivetrainConstants.driveGearRatio),
        getAngle());
  }

  /**
   * @return the setpoint the module is commanded to go to
   */
  @Logged(name = "Module Setpoint", importance = Importance.CRITICAL)
  public SwerveModuleState getSetpoint() {
    return setpoint;
  }

  /**
   * @return meters driven and angle of module
   */
  @Logged(name = "Module Position", importance = Importance.DEBUG)
  public SwerveModulePosition getPosition() {
    // BaseStatusSignal.getLatencyCompensatedValue(mDriveMotor.getPosition(),
    // mDriveMotor.getVelocity());
    return new SwerveModulePosition(
        Conversions.talonToMeters(
            mDriveMotor.getPosition().getValue(),
            DrivetrainConstants.wheelCircumference,
            DrivetrainConstants.driveGearRatio),
        getAngle());
  }
}
