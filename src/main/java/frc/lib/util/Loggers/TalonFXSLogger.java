// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.util.Loggers;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Fahrenheit;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.hardware.TalonFXS;
import edu.wpi.first.epilogue.CustomLoggerFor;
import edu.wpi.first.epilogue.Epilogue;
import edu.wpi.first.epilogue.Logged.Importance;
import edu.wpi.first.epilogue.logging.ClassSpecificLogger;
import edu.wpi.first.epilogue.logging.EpilogueBackend;

/** Add your docs here. */
@CustomLoggerFor(TalonFXS.class)
public class TalonFXSLogger extends ClassSpecificLogger<TalonFXS> {
  public TalonFXSLogger() {
    super(TalonFXS.class);
  }

  @Override
  public void update(EpilogueBackend logger, TalonFXS motor) {
    // list out faults
    // stator current
    if (Epilogue.shouldLog(Importance.DEBUG)) {
      logger.log("Device Temp Exceeded", motor.getFault_DeviceTemp().refresh().getValue());
      logger.log(
          "Forward Hard Limit Reached", motor.getFault_ForwardHardLimit().refresh().getValue());
      logger.log(
          "Forward Soft Limit Reached", motor.getFault_ForwardSoftLimit().refresh().getValue());
      logger.log(
          "Reverse Hard Limit Reached", motor.getFault_ReverseHardLimit().refresh().getValue());
      logger.log(
          "Reverse Soft Limit Reached", motor.getFault_ReverseSoftLimit().refresh().getValue());
      logger.log(
          "Fused Sensor Out of Sync", motor.getFault_FusedSensorOutOfSync().refresh().getValue());
      logger.log("Undervoltage Event", motor.getFault_Undervoltage().refresh().getValue());
      logger.log("Unstable supply voltage", motor.getFault_UnstableSupplyV().refresh().getValue());
      logger.log("Processor Temp Exceeded", motor.getFault_ProcTemp().refresh().getValue());
    }
    if (Epilogue.shouldLog(Importance.INFO) || Epilogue.shouldLog(Importance.DEBUG)) {
      logger.log("Velocity", motor.getVelocity().refresh().getValue().in(RotationsPerSecond));
      logger.log("Position", motor.getPosition().refresh().getValue().in(Rotations));
      logger.log("Output Voltage", motor.getMotorVoltage().refresh().getValue().in(Volts));
      logger.log("Device Temp", motor.getDeviceTemp().refresh().getValue().in(Fahrenheit));
    }
    // ! by default we always log critical information
    logger.log("Supply Current", motor.getSupplyCurrent().refresh().getValue().in(Amps));
    logger.log("Stator Current", motor.getStatorCurrent().refresh().getValue().in(Amps));

    logger.log("Hardware Fault", motor.getFault_Hardware().refresh().getValue());
    logger.log("OverVoltage Event", motor.getFault_OverSupplyV().refresh().getValue());
  }
}
