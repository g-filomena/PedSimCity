package pedsim.night.parameters;

public class NightPars {
    public enum DirectionalLuxStatistic {
        MIN,
        MEAN
    }

    public static DirectionalLuxStatistic directionalLuxStatistic = DirectionalLuxStatistic.MIN;
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

  // A/B twin testing is an opt-in experimental mode: when true it spawns abTestPairs vulnerable/
  // non-vulnerable twins instead of the census-derived population. Off by default so a normal run
  // uses the full sampled population.
  public static boolean enableLightABTesting = false;

  // Number of vulnerable/non-vulnerable twin pairs spawned in A/B mode (2 agents per pair).
  // User-configurable; independent of the census-derived population size.
  public static int abTestPairs = 72;

  // Upper bound on P(reroute) in NightBehaviour.rerouteOrIncreaseSpeed() as the current edge's
  // illuminance approaches 0 lux (full darkness). P(reroute) is 0.5 at or above the agent's
  // sensitivity threshold -- unchanged from the original fixed 50/50 split -- and rises linearly
  // toward this value as the edge gets darker below threshold. Replaces the single unmotivated
  // 0.5 literal register finding C5 flagged ("arbitrary and uncorrelated with how dark the edge
  // actually is") with the one parameter that behaviour actually needs: how strongly darkness
  // below threshold should tip the balance toward rerouting rather than just walking faster.
  // 0.9 is a starting value, not a calibrated one -- tune against the Torino/Lyon validation data.
  public static double maxRerouteProbabilityInDarkness = 0.9;
}
