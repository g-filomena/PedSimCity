package pedsim.core.server;

import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStreamReader;
import java.util.ArrayList;
import java.util.List;
import pedsim.core.parameters.ParameterManager;
import pedsim.core.utilities.LoggerUtil;

/**
 * Handles launching and stopping the simulation on a remote server via SSH.
 */
public class RemoteLauncher {

  // --- SSH / remote env config (defaults from server.properties; override via setters) ---
  private String sshPath = ServerConfig.sshExecutable();
  private String keyPath = ServerConfig.sshKey();
  private String server = ServerConfig.serverHost();

  // Project-specific remote config (editable after init)
  private String projectDir;
  private String mainClass;

  // Remote Java toolchain
  private String javaBinDir = ServerConfig.javaBinDir();
  private String classpath = ServerConfig.classpath();

  private String lastPid = null;

  public RemoteLauncher(ServerProjectConfig projectConfig) {
    this.projectDir = projectConfig.getProjectDir();
    this.mainClass = projectConfig.getMainClass();
  }

  // -------------------------
  // Getters & setters
  // -------------------------
  public String getSshPath() {
    return sshPath;
  }

  public void setSshPath(String sshPath) {
    this.sshPath = sshPath;
  }

  public String getKeyPath() {
    return keyPath;
  }

  public void setKeyPath(String keyPath) {
    this.keyPath = keyPath;
  }

  public String getServer() {
    return server;
  }

  public void setServer(String server) {
    this.server = server;
  }

  public String getProjectDir() {
    return projectDir;
  }

  public void setProjectDir(String projectDir) {
    this.projectDir = projectDir;
  }

  public String getMainClass() {
    return mainClass;
  }

  public void setMainClass(String mainClass) {
    this.mainClass = mainClass;
  }

  public String getJavaBinDir() {
    return javaBinDir;
  }

  public void setJavaBinDir(String javaBinDir) {
    this.javaBinDir = javaBinDir;
  }

  public String getClasspath() {
    return classpath;
  }

  public void setClasspath(String classpath) {
    this.classpath = classpath;
  }

  public String getLastPid() {
    return lastPid;
  }

  // -------------------------
  // Public API
  // -------------------------

  /**
   * Runs a simulation remotely over SSH.
   *
   * <p>Took a {@code PedSimCityApplet} until the AWT GUI was removed: it read the run's parameters
   * out of the panel's text fields and wrote progress back into the panel's log area. It now takes
   * the argument string directly and logs like everything else, which also means a remote run is
   * reproducible from what is written down rather than from what was typed into a window.
   *
   * @param fullArgs the command line to run on the server, e.g. {@code --headless --cityName=Torino}
   */
  public void runOnServer(String fullArgs) {
    String remoteCmd = buildRemoteCommand(fullArgs);

    LoggerUtil.getLogger().info("[SERVER][CMD] " + remoteCmd);

    try {
      ProcessBuilder pb = sshCommand(remoteCmd);
      pb.redirectErrorStream(true);
      Process proc = pb.start();

      new Thread(
              () -> {
                try (BufferedReader reader =
                    new BufferedReader(new InputStreamReader(proc.getInputStream()))) {
                  String line;
                  while ((line = reader.readLine()) != null) {
                    if (line.matches("\\d+")) {
                      lastPid = line.trim();
                      LoggerUtil.getLogger().info("[SERVER] PID: " + lastPid);
                    } else {
                      LoggerUtil.getLogger().info("[SERVER] " + line);
                    }
                  }
                } catch (Exception ex) {
                  LoggerUtil.getLogger().warning("Error reading server output: " + ex.getMessage());
                }
              })
          .start();

    } catch (IOException e) {
      LoggerUtil.getLogger().severe("SSH Error: " + e.getMessage());
    }
  }

  /**
   * Stops the remote simulation, by PID when one was captured and otherwise by main class.
   *
   * <p>The {@code pkill -f} fallback matches on the main class name, which also appears in the ssh
   * command line that carries it. Killing on a pattern that matches your own invocation kills the
   * shell running it - collect the PIDs in one call and kill them in the next.
   */
  public void stopOnServer() {
    String killCmd = lastPid != null ? "kill " + lastPid : "pkill -f " + mainClass;
    try {
      sshCommand(killCmd).start();
      LoggerUtil.getLogger().info("[SERVER] Sent kill command (" + killCmd + ")");
    } catch (IOException e) {
      LoggerUtil.getLogger().severe("SSH Error: " + e.getMessage());
    }
  }

  // -------------------------
  // Internals
  // -------------------------

  /** Build the ssh invocation, including {@code -i <key>} only when a key path is configured. */
  private ProcessBuilder sshCommand(String remoteCmd) {
    List<String> args = new ArrayList<>();
    args.add(sshPath);
    if (!keyPath.isBlank()) {
      args.add("-i");
      args.add(keyPath);
    }
    args.add(server);
    args.add(remoteCmd);
    return new ProcessBuilder(args);
  }

  /**
   * Build the full remote command chain (check JDK, pull, compile, run)
   */
  private String buildRemoteCommand(String fullArgs) {
    String java = javaBinDir.isBlank() ? "java" : javaBinDir + "/java";
    String javac = javaBinDir.isBlank() ? "javac" : javaBinDir + "/javac";

    return "echo '>> Checking Java version' && "
        + java
        + " -version && "
        + javac
        + " -version && "
        + "cd "
        + projectDir
        + " && "
        + "echo '>> pulling repo' && git pull && "
        + "echo '>> compiling sources' && mkdir -p bin && "
        + "find src/main/java -name '*.java' > sources.txt && "
        + javac
        + " -d bin -cp 'bin:lib/*' @sources.txt && "
        + "echo '>> running simulation' && "
        + java
        + " -XX:+UseNUMA -XX:+UseParallelGC -Xmx256G "
        + "-cp '"
        + classpath
        + "' "
        + mainClass
        + " "
        + fullArgs;
  }
}
