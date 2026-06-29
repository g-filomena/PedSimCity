package pedsim.night.parameters;

public class NightPars {
  // GUI Configurable parameters for light sensitivity (Lux)
  public static double minVulnerableLightSensitivity = 5.0;
  public static double maxVulnerableLightSensitivity = 15.0;
  public static double nonVulnerableLightSensitivity = 0.0;

  public static double crowdednessPercentile = 80.0;

  public static boolean enableLightABTesting = true;
}
