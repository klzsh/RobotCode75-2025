package frc.robot.subsystems.Drivetrain;

import static edu.wpi.first.units.Units.*;
import static frc.lib.math.Conversions.*;
import static frc.lib.util.CTREModuleState.optimize;
import static frc.robot.Constants.DrivetrainConstants.*;
import static frc.robot.Constants.DrivetrainConstants.MotorConfigs.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import frc.lib.util.SwerveModuleConstants;

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
// @Logged(strategy = Strategy.OPT_IN, importance = Importance.CRITICAL)
public class TalonFXSwerveModule {
  public int moduleNumber;
  private Rotation2d angleOffset;
  private Rotation2d lastAngle;

  // @Logged(name = "Angle Motor", importance = Importance.DEBUG)
  private TalonFX mAngleMotor;

  // @Logged(name = "Drive Motor", importance = Importance.DEBUG)
  private TalonFX mDriveMotor;

  // sets the "forward" position of the wheel
  // @Logged(name = "CanCoder", importance = Importance.DEBUG)
  private CANcoder angleEncoder;

  // used for logging
  private SwerveModuleState setpoint;

  /* drive motor control requests */
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
    this.angleOffset = moduleConstants.angleOffset();

    /* Angle Encoder Config */
    angleEncoder = new CANcoder(moduleConstants.cancoderID(), driveBusName);
    configAngleEncoder();

    /* Angle Motor Config */
    mAngleMotor = new TalonFX(moduleConstants.angleMotorID(), driveBusName);
    configAngleMotor();

    /* Drive Motor Config */
    mDriveMotor = new TalonFX(moduleConstants.driveMotorID(), driveBusName);
    configDriveMotor();

    lastAngle = getState().angle;
    setpoint = new SwerveModuleState();

    // set control requests to be one shot + enable timesync:
    driveDutyCycle.UpdateFreqHz = 0;
    driveDutyCycle.UseTimesync = true;

    torqueDrivevelocity.UpdateFreqHz = 0;
    torqueDrivevelocity.UseTimesync = true;

    torqueAngleCurrent.UpdateFreqHz = 0;
    torqueAngleCurrent.UseTimesync = true;

    torqueCharacterization.UpdateFreqHz = 0;
    torqueCharacterization.UseTimesync = true;
  }

  /**
   * sets the module state for an individual module
   *
   * @param desiredState the velocity and angle of the module
   * @param isOpenLoop Use duty cycle or closed loop control
   */
  public void setDesiredState(
      SwerveModuleState desiredState, boolean isOpenLoop, boolean steerWhenStationary) {
    /*
     * This is a custom optimize function, since default WPILib optimize assumes
     * continuous controller which CTRE and Rev onboard is not
     */

    desiredState = optimize(desiredState, getState().angle);
    setpoint = desiredState;
    setAngle(desiredState, steerWhenStationary);
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
      driveDutyCycle.Output = desiredState.speedMetersPerSecond / maxSpeed.in(MetersPerSecond);
      mDriveMotor.setControl(driveDutyCycle.withEnableFOC(true));
    } else {
      mDriveMotor.setControl(
          torqueDrivevelocity.withVelocity(
              MPSToTalon(
                  MetersPerSecond.of(desiredState.speedMetersPerSecond),
                  wheelCircumference,
                  driveGearRatio)));
    }
  }

  /**
   * Used to determine the amount of current to overcome the static friction between the ground and
   * the module wheels.
   *
   * @param current the current to apply to the drive motor
   */
  public void characterizeDrive(Current current) {
    mDriveMotor.setControl(torqueCharacterization.withOutput(current));
  }

  /**
   * Sets the angle of the module via position control
   *
   * @param desiredState the velocity and angle of the module (only uses angle)
   * @param steerWhenStationary used for XStance
   */
  private void setAngle(SwerveModuleState desiredState, boolean steerWhenStationary) {
    Rotation2d angle = new Rotation2d();
    if (!steerWhenStationary) {
      angle =
          (Math.abs(desiredState.speedMetersPerSecond) <= (maxSpeed.in(MetersPerSecond) * 0.01))
              ? lastAngle
              : desiredState
                  .angle; // Prevent rotating module if speed is less then 1%. Prevents Jittering.
    } else {
      angle = desiredState.angle;
    }

    mAngleMotor.setControl(
        torqueAngleCurrent.withPosition(
            degreesToTalon(Degrees.of(angle.getDegrees()), angleGearRatio)));

    lastAngle = angle;
  }

  /**
   * @return the current angle of the angle motor
   */
  // @Logged(name = "Module Angle", importance = Importance.DEBUG)
  public Rotation2d getAngle() {
    Measure<AngleUnit> LatencyCompensatedPosition =
        BaseStatusSignal.getLatencyCompensatedValue(
            mAngleMotor.getPosition().refresh(), mAngleMotor.getVelocity().refresh());

    return Rotation2d.fromDegrees(
        talonToDegrees(Rotations.of(LatencyCompensatedPosition.in(Rotations)), angleGearRatio)
            .in(Degrees));
  }

  /**
   * @return CANCoder angle
   */
  public Rotation2d getCANCoder() {
    return Rotation2d.fromRotations(angleEncoder.getAbsolutePosition().getValue().in(Rotations));
  }

  // @Logged(name = "CANCoder angle", importance = Importance.DEBUG)
  public double logCanCoderDegrees() {
    return angleEncoder.getAbsolutePosition().refresh().getValue().in(Degrees);
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
        degreesToTalon(
            Degrees.of(waitForCANcoder().getDegrees() - angleOffset.getDegrees()), angleGearRatio);
    mAngleMotor.setPosition(absolutePosition);
  }

  /** configures angle encoder based on ctreConfigs */
  private void configAngleEncoder() {
    angleEncoder.getConfigurator().apply(getEncoderConfiguration());
  }

  /** configures angle motor based on ctreConfigs */
  private void configAngleMotor() {
    mAngleMotor.getConfigurator().apply(getAngleConfiguration());
    resetToAbsolute();
  }

  /** configures drive motor based on ctreConfigs */
  private void configDriveMotor() {
    mDriveMotor.getConfigurator().apply(getDriveConfiguration());
    mDriveMotor.getConfigurator().setPosition(0);
  }

  public void setDrivePIDS(Slot0Configs config) {
    mDriveMotor.getConfigurator().apply(config);
  }

  public void setAnglePIDS(Slot0Configs config) {
    mAngleMotor.getConfigurator().apply(config);
  }

  /**
   * @return the velocity and angle of the module
   */
  // @Logged(name = "Module State", importance = Importance.DEBUG)
  public SwerveModuleState getState() {
    return new SwerveModuleState(
        talonToMPS(mDriveMotor.getVelocity().getValue(), wheelCircumference, driveGearRatio),
        getAngle());
  }

  /**
   * @return the setpoint the module is commanded to go to
   */
  // @Logged(name = "Module Setpoint", importance = Importance.DEBUG)
  public SwerveModuleState getSetpoint() {
    return setpoint;
  }

  /**
   * @return meters driven and angle of module
   */
  // @Logged(name = "Module Position", importance = Importance.DEBUG)
  public SwerveModulePosition getPosition() {
    Measure<AngleUnit> LatencyCompensatedPosition =
        BaseStatusSignal.getLatencyCompensatedValue(
            mDriveMotor.getPosition().refresh(), mDriveMotor.getVelocity().refresh());

    return new SwerveModulePosition(
        talonToMeters(
            Rotations.of(LatencyCompensatedPosition.in(Rotations)),
            wheelCircumference,
            driveGearRatio),
        getAngle());
  }
}
