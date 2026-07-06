package pedsim.night.parameters;

public class NightPars {
  // GUI Configurable parameters for light sensitivity (Lux)
  public static double minVulnerableLightSensitivity = 5.0;
  public static double maxVulnerableLightSensitivity = 15.0;
  // Non-vulnerable agents treat an edge as dark only below 5 lux (the pipeline's UNLIT_LUX_THRESHOLD).
  public static double nonVulnerableLightSensitivity = 5.0;

  // Nominal illuminance (lux) credited to edges known lit only via the binary "lit" flag (no
  // continuous mean_lux), for the per-agent lux metric. Defaults to the lit/unlit threshold as a
  // conservative lower bound (a lit edge is at least this bright); raise for a more representative
  // lit-street value.
  public static double litEdgeNominalLux = 5.0;

  public static double crowdednessPercentile = 80.0;

  public static boolean enableLightABTesting = true;
}
