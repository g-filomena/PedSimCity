package pedsim.empirical.engine;

import pedsim.core.engine.PedSimCity;
import pedsim.core.engine.ScenarioConfig;
import pedsim.core.parameters.Pars;
import pedsim.empirical.agent.EmpiricalGroup;
import pedsim.empirical.parameters.EmpiricalPars;

/** Empirical ABM simulation state. */
public class PedSimCityEmpirical extends PedSimCity {

  private static final long serialVersionUID = 1L;

  public PedSimCityEmpirical(long seed, int job, ScenarioConfig scenarioConfig) {
    super(seed, job, defaultScenarioConfig(scenarioConfig));
  }

  private static ScenarioConfig defaultScenarioConfig(ScenarioConfig scenarioConfig) {
    if (scenarioConfig != null) {
      return scenarioConfig;
    }

    return new ScenarioConfig(EmpiricalGroup.values(), null);
  }

  @Override
  protected void populateEnvironment() {
    EmpiricalPopulate populate = new EmpiricalPopulate();
    populate.populateEmpiricalGroups(this);
  }

}
