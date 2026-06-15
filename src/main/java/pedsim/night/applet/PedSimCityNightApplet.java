package pedsim.night.applet;

import java.awt.event.WindowAdapter;
import java.awt.event.WindowEvent;
import pedsim.core.applet.PedSimCityApplet;
import pedsim.core.applet.ServerProjectConfig;
import pedsim.core.engine.Engine;
import pedsim.core.engine.ScenarioConfig;
import pedsim.core.engine.SimulationLauncher;
import pedsim.core.parameters.Pars;
import pedsim.core.utilities.LoggerUtil;
import pedsim.core.utilities.StringEnum;
import pedsim.core.website.SimulationRestApi;
import pedsim.night.engine.NightEngine;
import pedsim.night.engine.NightSimulationModule;
import pedsim.night.engine.PedSimCityNight;

public class PedSimCityNightApplet extends PedSimCityApplet {

  private static final long serialVersionUID = 1L;

  private NightParametersPanel nightPanel;

  public PedSimCityNightApplet() {
    super();
    Pars.isNight = true;

    routeParsButton.setVisible(true);
    routeParsButton.setLabel("Night Parameters");

    for (java.awt.event.ActionListener al : routeParsButton.getActionListeners()) {
      routeParsButton.removeActionListener(al);
    }

    routeParsButton.addActionListener(e -> openNightPanel());
  }

  private void openNightPanel() {
    if (nightPanel == null || !nightPanel.isDisplayable()) {
      nightPanel = new NightParametersPanel();
    }
    nightPanel.setVisible(true);
  }

  /** Returns a {@link SimulationLauncher} configured for night mode. */
  private static SimulationLauncher nightLauncher() {
    return new SimulationLauncher(NightSimulationModule.INSTANCE);
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
      LoggerUtil.getLogger().info("[SERVER] Running headless night simulation...");
      nightLauncher().headlessRun(args);
      return;
    }

    int choice = 0;
    if (forceWebsite) choice = 2;
    else if (forceGui) choice = 1;
    else {
      System.out.println("\n========================================");
      System.out.println("   PedSimCity Night - Startup Options   ");
      System.out.println("========================================");
      System.out.println(" 1. Run Standard GUI (AWT)");
      System.out.println(" 2. Start REST API for External Dashboard (Streamlit)");
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
      SimulationLauncher launcher = nightLauncher();
      launcher.preloadForDashboard();
      launcher.wireAndStartRestServer(8081);
    } else {
      LoggerUtil.getLogger().info("[STARTUP] Launching Standard GUI...");

      // REST server starts for state observation only; no setOnStart registered
      // so /api/start returns 503. Simulations are triggered via the GUI button.
      SimulationRestApi.start(8081);

      PedSimCityNightApplet applet = new PedSimCityNightApplet();
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
    return "PedSimCity Night Applet";
  }

  @Override
  protected void updateCityNameOptions() {
    cityName.removeAll();
    cityName.add("Torino");
    cityName.validate();
  }

  @Override
  protected ServerProjectConfig buildServerProjectConfig() {
    return new ServerProjectConfig(
        "/mnt/home/gabriele/PedSimCityNight",
        "pedsim.night.applet.PedSimCityNightApplet",
        "bin:lib/*:src/main/resources");
  }

  @Override
  protected ScenarioConfig buildScenarioConfig() {
    return new ScenarioConfig(StringEnum.Vulnerable.values(), StringEnum.TimeOfDay.values());
  }

  @Override
  protected Engine.StateFactory buildStateFactory() {
    return PedSimCityNight::new;
  }

  @Override
  protected Engine buildEngine() {
    return new NightEngine(buildStateFactory());
  }
}
