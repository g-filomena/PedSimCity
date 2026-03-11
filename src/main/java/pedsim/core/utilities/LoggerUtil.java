package pedsim.core.utilities;

import java.awt.TextArea;
import java.util.logging.ConsoleHandler;
import java.util.logging.Formatter;
import java.util.logging.Handler;
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

  /**
   * Redirect logger output also to a TextArea in the GUI.
   */
  public static void redirectToTextArea(TextArea textArea) {
    Handler guiHandler = new Handler() {
      @Override
      public void publish(LogRecord record) {
        if (textArea != null && isLoggable(record)) {
          textArea.append(record.getLevel() + ": " + record.getMessage() + "\n");
        }
      }

      @Override
      public void flush() {}

      @Override
      public void close() throws SecurityException {}
    };
    guiHandler.setLevel(Level.ALL);

    logger.addHandler(guiHandler);
  }
}
