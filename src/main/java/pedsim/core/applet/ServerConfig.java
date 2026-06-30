package pedsim.core.applet;

import java.io.IOException;
import java.io.InputStream;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.Properties;
import pedsim.core.utilities.LoggerUtil;

/**
 * Server-launch configuration, read from a user-supplied properties file so that ssh / key / host
 * and remote paths are not hardcoded in the source.
 *
 * <p>Lookup order: {@code server.properties} in the working directory (the repo root — the
 * recommended place to edit), then a bundled {@code /server.properties} on the classpath, then
 * built-in defaults. Copy {@code server.properties.example} to {@code server.properties} and fill in
 * your own values. Every key is optional; a blank or missing value falls back to a sensible default.
 */
public final class ServerConfig {

  private static final Properties PROPS = load();

  private ServerConfig() {}

  private static Properties load() {
    Properties p = new Properties();

    Path local = Path.of("server.properties");
    if (Files.exists(local)) {
      try (var in = Files.newInputStream(local)) {
        p.load(in);
        return p;
      } catch (IOException e) {
        LoggerUtil.getLogger().warning("Could not read server.properties: " + e.getMessage());
      }
    }

    try (InputStream in = ServerConfig.class.getResourceAsStream("/server.properties")) {
      if (in != null) {
        p.load(in);
      } else {
        LoggerUtil.getLogger()
            .info("No server.properties found; using defaults. Copy server.properties.example.");
      }
    } catch (IOException e) {
      // fall through to defaults
    }
    return p;
  }

  private static String get(String key, String def) {
    String v = PROPS.getProperty(key);
    return (v == null || v.isBlank()) ? def : v.trim();
  }

  /** SSH client executable; defaults to {@code ssh} on the PATH. */
  public static String sshExecutable() {
    return get("ssh.executable", "ssh");
  }

  /** Private key path; blank to rely on the ssh agent / default key. */
  public static String sshKey() {
    return get("ssh.key", "");
  }

  /** Remote server as {@code user@host}. */
  public static String serverHost() {
    return get("server.host", "");
  }

  /** Remote JDK {@code bin} directory; blank to use {@code java}/{@code javac} from the PATH. */
  public static String javaBinDir() {
    return get("server.javaBinDir", "");
  }

  /** Classpath used for the remote run. */
  public static String classpath() {
    return get("server.classpath", "bin:lib/*:src/main/resources");
  }

  /** Remote base directory under which each module's checkout lives; blank for an SSH-relative path. */
  public static String projectBaseDir() {
    return get("server.projectBaseDir", "");
  }

  /** Remote project directory for a module, e.g. {@code <projectBaseDir>/PedSimCityNight}. */
  public static String remoteProjectDir(String moduleDir) {
    String base = projectBaseDir();
    return base.isEmpty() ? moduleDir : base + "/" + moduleDir;
  }
}
