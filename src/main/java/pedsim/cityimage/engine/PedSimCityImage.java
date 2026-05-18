package pedsim.cityimage.engine;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import org.locationtech.jts.linearref.LengthIndexedLine;
import org.locationtech.jts.planargraph.DirectedEdge;
import pedsim.core.engine.PedSimCity;
import pedsim.core.engine.ScenarioConfig;

/**
 * The PedSimCity class represents the main simulation environment.
 */
public class PedSimCityImage extends PedSimCity {

  private static final long serialVersionUID = 1L;

  public PedSimCityImage(long seed, int job, ScenarioConfig scenarioConfig) {
    super(seed, job, scenarioConfig);
  }

  // OD related variables
  public static List<Float> distances = new ArrayList<>();
  public static final Map<DirectedEdge, LengthIndexedLine> indexedEdgeCache = new HashMap<>();

  /**
   * Populates the simulation environment with agents and other entities based on the selected
   * simulation parameters. This method uses the Populate class to generate the agent population.
   */
  @Override
  protected void populateEnvironment() {
    Populate populate = new Populate();
    populate.populateTests(this);
  }

  // /**
  // * The main function that allows the simulation to be run in stand-alone, non-GUI mode.
  // *
  // * @param args Command-line arguments.
  // * @throws Exception If an error occurs during simulation execution.
  // */
  // public static void main(String[] args) throws Exception {
  //
  // TestPars.defineMode();
  // Import importer = new Import();
  // importer.importFiles();
  // Environment.prepare();
  //
  // for (int job = 0; job < TestPars.jobs; job++) {
  // System.out.println("Run nr.. " + job);
  // final SimState state = new PedSimCity(System.currentTimeMillis(), job);
  // state.start();
  // ((PedSimCity) state).startSchedule();
  // while (state.schedule.step(state)) {
  // }
  // }
  // System.exit(0);
  // }
}
