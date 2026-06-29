package pedsim.activity.applet;

import java.awt.event.WindowAdapter;
import java.awt.event.WindowEvent;
import pedsim.activity.engine.ActivityEngine;
import pedsim.activity.engine.ActivitySimulationModule;
import pedsim.activity.engine.PedSimCityActivity;
import pedsim.core.applet.PedSimCityApplet;
import pedsim.core.applet.ServerProjectConfig;
import pedsim.core.engine.Engine;
import pedsim.core.engine.ScenarioConfig;
import pedsim.core.engine.SimulationLauncher;
import pedsim.core.parameters.Pars;
import pedsim.core.utilities.LoggerUtil;
import pedsim.core.utilities.StringEnum;
import pedsim.core.website.SimulationRestApi;

/**
 * Entry point for the activity-based module. Mirrors {@code PedSimCityNightApplet} /
 * {@code PedSimCityLearningApplet}: reuses the core AWT applet for the GUI and routes the
 * headless/website paths through a {@link SimulationLauncher} bound to
 * {@link ActivitySimulationModule}.
 */
public class PedSimCityActivityApplet extends PedSimCityApplet {

  private static final long serialVersionUID = 1L;

  public PedSimCityActivityApplet() {
    super();
    Pars.isNight = false;
  }

  /** Returns a {@link SimulationLauncher} configured for the activity module. */
  private static SimulationLauncher activityLauncher() {
    return new SimulationLauncher(ActivitySimulationModule.INSTANCE);
  }

  public static void main(String[] args) throws Exception {
    boolean headless = false;
    boolean forceWebsite = false;
    boolean forceGui = false;

    for (String arg : args) {
      if ("--headless".equals(arg) || arg.startsWith("--headless=")) headless = true;
      if ("--website".equals(arg)) forceWebsite = true;
      if ("--gui".equals(arg)) forceGui = true;
    }

    if (headless) {
      LoggerUtil.getLogger().info("[SERVER] Running headless activity simulation...");
      activityLauncher().headlessRun(args);
      return;
    }

    int choice = 0;
    if (forceWebsite) choice = 2;
    else if (forceGui) choice = 1;
    else {
      System.out.println("\n--- PedSimCity Activity ---");
      System.out.println("Select interface mode:");
      System.out.println("  1) Standard Applet GUI");
      System.out.println("  2) Web Dashboard (Browser)");
      System.out.print("\nSelect an option [1-2]: ");

      java.util.Scanner scanner = new java.util.Scanner(System.in);
      try {
        choice = Integer.parseInt(scanner.nextLine().trim());
      } catch (Exception e) {
        choice = 1;
      }
    }

    if (choice == 2) {
      LoggerUtil.getLogger().info("[STARTUP] Starting REST API for External Dashboard...");
      SimulationLauncher launcher = activityLauncher();
      launcher.preloadForDashboard();
      launcher.wireAndStartRestServer(8081);
      SimulationRestApi.openDashboardInBrowser();
    } else {
      LoggerUtil.getLogger().info("[STARTUP] Launching Standard GUI...");

      // REST server starts for state observation only; simulations are triggered via the GUI button.
      SimulationRestApi.start(8081);

      PedSimCityActivityApplet applet = new PedSimCityActivityApplet();
      applet.addWindowListener(
          new WindowAdapter() {
            @Override
            public void windowClosing(WindowEvent e) {
              applet.dispose();
            }
          });
    }
  }

  @Override
  protected String getAppletTitle() {
    return "PedSimCity Activity Applet";
  }

  @Override
  protected void updateCityNameOptions() {
    cityName.removeAll();
    cityName.add("Melbourne");
    cityName.add("Torino");
    cityName.validate();
  }

  @Override
  protected ServerProjectConfig buildServerProjectConfig() {
    return new ServerProjectConfig(
        "/mnt/home/gabriele/PedSimCityActivity",
        "pedsim.activity.applet.PedSimCityActivityApplet",
        "bin:lib/*:src/main/resources");
  }

  @Override
  protected ScenarioConfig buildScenarioConfig() {
    return new ScenarioConfig(StringEnum.Default.values(), StringEnum.Hour.values());
  }

  @Override
  protected Engine.StateFactory buildStateFactory() {
    return PedSimCityActivity::new;
  }

  @Override
  protected Engine buildEngine() {
    return new ActivityEngine(buildStateFactory());
  }
}
