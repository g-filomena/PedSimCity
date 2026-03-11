package pedsim.core.engine;


public final class ScenarioConfig {
  private final Enum<?>[] agentScenarioValues;
  private final Enum<?>[] simulationScenarioValues;

  public ScenarioConfig(Enum<?>[] agentScenarioValues, Enum<?>[] simulationScenarioValues) {
    this.agentScenarioValues = agentScenarioValues;
    this.simulationScenarioValues = simulationScenarioValues;
  }

  public Enum<?>[] getAgentScenarioValues() {
    return agentScenarioValues;
  }

  public Enum<?>[] getSimulationScenarioValues() {
    return simulationScenarioValues;
  }
}


