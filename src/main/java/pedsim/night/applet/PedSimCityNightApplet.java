package pedsim.night.applet;

import java.awt.event.WindowAdapter;
import java.awt.event.WindowEvent;
import pedsim.core.applet.PedSimCityApplet;
import pedsim.core.applet.ServerProjectConfig;
import pedsim.core.engine.Engine;
import pedsim.core.engine.ScenarioConfig;
import pedsim.core.parameters.ParameterManager;
import pedsim.core.parameters.Pars;
import pedsim.core.utilities.LoggerUtil;
import pedsim.core.utilities.StringEnum;
import pedsim.night.engine.PedSimCityNight;

/**
 * A graphical user interface (GUI) applet for configuring and running the PedSimCity simulation.
 * This applet allows users to select simulation parameters, start the simulation, and view
 * simulation progress. It provides options for choosing the simulation mode, city name, and other
 * simulation-specific settings. Users can also enable specific origin-destination (OD) testing and
 * access other advanced options.
 */
public class PedSimCityNightApplet extends PedSimCityApplet {

  private static final long serialVersionUID = 1L;

  public PedSimCityNightApplet() {
    super();
    routeParsButton.setVisible(false);
  }

  public static void main(String[] args) throws Exception {
    boolean headless = false;

    for (String arg : args) {
      if ("--headless".equals(arg) || arg.startsWith("--headless=")) {
        headless = true;
        break;
      }
    }

    if (headless) {
      LoggerUtil.getLogger().info("[SERVER] Running headless night simulation...");
      ParameterManager.initFromArgsForServer(args);

      ScenarioConfig scenarioConfig =
          new ScenarioConfig(StringEnum.Vulnerable.values(), StringEnum.TimeOfDay.values());
      Engine.StateFactory stateFactory = PedSimCityNight::new;

      Engine engine = new Engine(stateFactory);
      engine.runJobs(scenarioConfig, Pars.parallel);

    } else {
      PedSimCityNightApplet applet = new PedSimCityNightApplet();
      applet.addWindowListener(new WindowAdapter() {
        @Override
        public void windowClosing(WindowEvent e) {
          applet.dispose();
        }
      });
    }
  }

  @Override
  protected String getAppletTitle() {
    return "PedSimCity Night Applet";
  }

  @Override
  protected void updateCityNameOptions() {
    cityName.removeAll();
    cityName.add("TorinoCentre");
    cityName.validate();
  }

  @Override
  protected ServerProjectConfig buildServerProjectConfig() {
    return new ServerProjectConfig("/mnt/home/gabriele/PedSimCityNight",
        "pedsim.night.applet.PedSimCityNightApplet", "bin:lib/*:src/main/resources");
  }

  @Override
  protected ScenarioConfig buildScenarioConfig() {
    return new ScenarioConfig(StringEnum.Vulnerable.values(), StringEnum.TimeOfDay.values());
  }

  @Override
  protected Engine.StateFactory buildStateFactory() {
    return PedSimCityNight::new;
  }
}

