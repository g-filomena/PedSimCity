package pedsim.empirical.launcher;

import pedsim.core.engine.ScenarioConfig;
import pedsim.core.parameters.ParameterManager;
import pedsim.core.parameters.Pars;
import pedsim.core.utilities.LoggerUtil;
import pedsim.empirical.agent.EmpiricalGroup;
import pedsim.empirical.engine.EmpiricalEngine;
import pedsim.empirical.engine.PedSimCityEmpirical;
import pedsim.empirical.parameters.EmpiricalPars;

/**
 * Entry point for the empirical ABM module.
 *
 * <p>This module had no headless entry at all: {@code PedSimCityEmpiricalApplet.main} opened a
 * window and nothing else, so the only way to run it without a display was
 * {@code PedSimCityEmpirical.main}, which took the city as a bare positional argument and reached no
 * parameter handling whatsoever. Both are replaced by this, which takes {@code --key=value} like
 * every other module.
 *
 * <p>Like cityImage, empirical implements no {@link pedsim.core.engine.SimulationModule}, so it
 * reads no per-city configuration file and its own {@code EmpiricalPars} keys are not reachable from
 * the command line. See {@code CityImageLauncher} and NEXT.md.
 */
public final class EmpiricalLauncher {

  private EmpiricalLauncher() {}

  public static void main(String[] args) throws Exception {
    LoggerUtil.getLogger().info("[RUN] empirical simulation, headless.");
    EmpiricalPars.applyDefaults();
    ParameterManager.initFromArgs(args);

    ScenarioConfig scenarioConfig = new ScenarioConfig(EmpiricalGroup.values(), null);
    new EmpiricalEngine(PedSimCityEmpirical::new).runJobs(scenarioConfig, Pars.parallel);
  }
}
