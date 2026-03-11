package pedsim.core.applet;

public class ServerProjectConfig {

  private final String projectDir;
  private final String mainClass;
  private final String classpath;

  public ServerProjectConfig(String projectDir, String mainClass, String classpath) {
    this.projectDir = projectDir;
    this.mainClass = mainClass;
    this.classpath = classpath;
  }

  public String getProjectDir() {
    return projectDir;
  }

  public String getMainClass() {
    return mainClass;
  }

  public String getClasspath() {
    return classpath;
  }
}
