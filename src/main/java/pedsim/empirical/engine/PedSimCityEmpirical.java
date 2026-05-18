package pedsim.empirical.engine;

import java.util.ArrayList;
import java.util.List;
import pedsim.cityimage.parameters.TestPars;
import pedsim.core.engine.PedSimCity;
import pedsim.core.engine.ScenarioConfig;
import pedsim.empirical.agent.EmpiricalAgentsGroup;
import sim.engine.SimState;
import sim.engine.Stoppable;

/**
 * The PedSimCity class represents the main simulation environment.
 */
public class PedSimCityEmpirical extends PedSimCity {


  private static final long serialVersionUID = 1L;

  public PedSimCityEmpirical(long seed, int job, ScenarioConfig scenarioConfig) {
    super(seed, job, scenarioConfig);
  }

  public static List<EmpiricalAgentsGroup> empiricalGroups = new ArrayList<>();

  /**
   * Initialises the simulation by defining the simulation mode, initialising edge volumes, and
   * preparing the simulation environment. It then proceeds to populate the environment with agents
   * and starts the agent movement.
   */
  // @Override
  // public void start() {
  // super.start();
  // prepareEnvironment();
  // populateEnvironment();
  // }

  /**
   * Populates the simulation environment with agents and other entities based on the selected
   * simulation parameters. This method uses the Populate class to generate the agent population.
   */
  @Override
  protected void populateEnvironment() {
    Populate populate = new Populate();
    populate.populateEmpiricalGroups(this);
  }

  /**
   * Starts moving agents in the simulation. This method schedules agents for repeated movement
   * updates and sets up the spatial index for agents.
   */
  public void startSchedule() {
    for (Agent agent : this.agentsList) {
      Stoppable stop = schedule.scheduleRepeating(agent);
      agent.setStoppable(stop);
      schedule.scheduleRepeating(agents.scheduleSpatialIndexUpdater(), Integer.MAX_VALUE, 1.0);
    }
    agents.setMBR(MBR);
  }

  /**
   * The main function that allows the simulation to be run in stand-alone, non-GUI mode.
   *
   * @param args Command-line arguments.
   * @throws Exception If an error occurs during simulation execution.
   */
  public static void main(String[] args) throws Exception {

    TestPars.defineMode();
    Import importer = new Import();
    importer.importFiles();
    Environment.prepare();

    for (int job = 0; job < TestPars.jobs; job++) {
      System.out.println("Run nr.. " + job);
      final SimState state = new PedSimCity(System.currentTimeMillis(), job);
      state.start();
      ((PedSimCity) state).startSchedule();
      while (state.schedule.step(state)) {
      }
    }
    System.exit(0);
  }
}
