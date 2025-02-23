package frc.lib.util.Loggers;

import edu.wpi.first.epilogue.CustomLoggerFor;
import edu.wpi.first.epilogue.logging.ClassSpecificLogger;
import edu.wpi.first.epilogue.logging.EpilogueBackend;
import java.util.OptionalDouble;

@CustomLoggerFor(OptionalDouble.class)
public class OptionalDoubleLogger extends ClassSpecificLogger<OptionalDouble> {
  public OptionalDoubleLogger() {
    super(OptionalDouble.class);
  }

  @Override
  public void update(EpilogueBackend logger, OptionalDouble val) {
    logger.log("value", val.isPresent() ? val.getAsDouble() : null);
  }
}
