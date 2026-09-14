package pedsim.night.parameters;

public class NightPars {
    public enum DirectionalLuxStatistic {
        MIN,
        MEAN
    }

    // MEAN, not MIN (register finding B1). The 12 m visibility horizon is well supported --
    // Fotios, Yang & Uttley (2015) measured pedestrian fixation on other people at 10.3 m,
    // 15 m as the recommended observation distance -- but MIN across that window is about one
    // lamp spacing wide (mean nearest-neighbour spacing 11.7 m; 39% of lamps have no neighbour
    // within 12 m), so it reduces to "how far is the entry node from the nearest lamp", a
    // property of where the junction happens to fall in the lighting rhythm rather than of the
    // street. Switching to MEAN relabelled 4.8% of entrances from dark to lit when measured
    // against the pipeline's own directional lookup, and both visibility_mean_lux and
    // visibility_min_lux are already written by 04_directional_lighting.py, so this is a
    // same-run NightImport column switch, not a pipeline re-run.
    public static DirectionalLuxStatistic directionalLuxStatistic = DirectionalLuxStatistic.MEAN;
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

  // Ceiling on the route-planning cost multiplier DijkstraRoadDistanceNight applies to a KNOWN
  // edge whose mean_lux falls below the travelling agent's own lightSensitivityThreshold
  // (register finding C2: "light never enters route planning"). 1.0 = no penalty (plain distance
  // x perception error, as before this fix); this value = the multiplier at 0 lux on a fully
  // known edge. Restricted to known edges only -- an unknown edge's darkness stays entirely a
  // situated-reaction matter, so this and maxRerouteProbabilityInDarkness never double-count the
  // same darkness. 1.5 is a starting value, not a calibrated one -- tune against the Torino/Lyon
  // validation data.
  public static double maxKnownDarkEdgeCostMultiplier = 1.5;
}
