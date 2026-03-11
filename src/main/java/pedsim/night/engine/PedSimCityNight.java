package pedsim.night.engine;

import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Set;
import java.util.concurrent.ConcurrentHashMap;
import org.javatuples.Pair;
import org.locationtech.jts.linearref.LengthIndexedLine;
import org.locationtech.jts.planargraph.DirectedEdge;
import pedsim.core.engine.PedSimCity;
import pedsim.core.engine.ScenarioConfig;
import sim.graph.EdgeGraph;
import sim.graph.NodeGraph;

/**
 * The PedSimCity class represents the main simulation environment.
 */
public class PedSimCityNight extends PedSimCity {

  private static final long serialVersionUID = 1L;
  public boolean isDark = false;
  public static final Map<DirectedEdge, LengthIndexedLine> indexedEdgeCache = new HashMap<>();

  public static Set<EdgeGraph> edges = new HashSet<>();

  // cached route
  public static Map<Pair<NodeGraph, NodeGraph>, List<DirectedEdge>> routesDay =
      new ConcurrentHashMap<>();
  public static Map<Pair<NodeGraph, NodeGraph>, List<DirectedEdge>> routesNonVulnerableNight =
      new ConcurrentHashMap<>();
  public static Map<Pair<NodeGraph, NodeGraph>, List<DirectedEdge>> routesVulnerableNight =
      new ConcurrentHashMap<>();

  // cached alternative routes for night movement
  public static Map<Pair<NodeGraph, NodeGraph>, List<DirectedEdge>> altRoutesVulnerable =
      new ConcurrentHashMap<>();
  public static Map<Pair<NodeGraph, NodeGraph>, List<DirectedEdge>> altRoutesNonVulnerable =
      new ConcurrentHashMap<>();

  /**
   * Constructs a new instance of the PedSimCity simulation environment.
   *
   * @param seed The random seed for the simulation.
   * @param job The current job number for multi-run simulations.
   */
  public PedSimCityNight(long seed, int job, ScenarioConfig scenarioConfig) {
    super(seed, job, scenarioConfig);
  }

  /**
   * Populates the simulation environment with agents and other entities based on the selected
   * simulation parameters. This method uses the Populate class to generate the agent population.
   */
  @Override
  protected void populateEnvironment() {
    Populate populate = new Populate();
    populate.populate(this);
  }
}
