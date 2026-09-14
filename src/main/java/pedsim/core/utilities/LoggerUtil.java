package pedsim.core.utilities;

import java.util.logging.ConsoleHandler;
import java.util.logging.Formatter;
import java.util.logging.Level;
import java.util.logging.LogRecord;
import java.util.logging.Logger;

public class LoggerUtil {

  // --- Simple formatter (console only) ---
  public static class SimpleFormatter extends Formatter {
    @Override
    public String format(LogRecord record) {
      return record.getLevel() + ": " + record.getMessage() + "\n";
    }
  }

  private static final Logger logger = Logger.getLogger(LoggerUtil.class.getName());

  static {
    // Remove all existing handlers
    Logger rootLogger = Logger.getLogger("");
    for (var handler : rootLogger.getHandlers()) {
      rootLogger.removeHandler(handler);
    }

    // Console handler with custom formatter
    ConsoleHandler handler = new ConsoleHandler();
    handler.setFormatter(new SimpleFormatter());
    rootLogger.addHandler(handler);

    // Set log level
    rootLogger.setLevel(Level.INFO);
    logger.setLevel(Level.INFO);
  }

  public static Logger getLogger() {
    return logger;
  }

  // redirectToTextArea(TextArea) lived here, mirroring the log into the applet's log pane. It went
  // with the AWT GUI: the console and the run's own output files are the record now.
}
