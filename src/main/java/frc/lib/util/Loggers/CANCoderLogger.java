// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.util.Loggers;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.hardware.CANcoder;
import edu.wpi.first.epilogue.CustomLoggerFor;
import edu.wpi.first.epilogue.Epilogue;
import edu.wpi.first.epilogue.Logged.Importance;
import edu.wpi.first.epilogue.logging.ClassSpecificLogger;
import edu.wpi.first.epilogue.logging.DataLogger;

/** Add your docs here. */
@CustomLoggerFor(CANcoder.class)
public class CANCoderLogger extends ClassSpecificLogger<CANcoder> {
  public CANCoderLogger() {
    super(CANcoder.class);
  }

  @Override
  public void update(DataLogger logger, CANcoder CANcoder) {
    if (Epilogue.shouldLog(Importance.INFO)) {
      logger.log("Absolute Position", CANcoder.getAbsolutePosition().getValue().in(Degrees));
      logger.log("Magnet Fault", CANcoder.getFault_BadMagnet().getValue());
    }
    logger.log("Velocity", CANcoder.getVelocity().refresh().getValue().in(RotationsPerSecond));
    logger.log("Position", CANcoder.getPosition().refresh().getValue().in(Degrees));
  }
}
