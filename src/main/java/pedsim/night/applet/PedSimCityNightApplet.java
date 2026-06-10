package pedsim.night.applet;

import java.awt.Desktop;
import java.awt.event.WindowAdapter;
import java.awt.event.WindowEvent;
import java.net.URI;

import pedsim.core.applet.PedSimCityApplet;
import pedsim.core.applet.ServerProjectConfig;
import pedsim.core.engine.Engine;
import pedsim.core.engine.ScenarioConfig;
import pedsim.core.parameters.ParameterManager;
import pedsim.core.parameters.Pars;
import pedsim.core.utilities.LoggerUtil;
import pedsim.core.utilities.StringEnum;
import pedsim.core.website.SimulationRestApi;
import pedsim.night.engine.PedSimCityNight;

/**
 * A graphical user interface (GUI) applet for configuring and running the
 * PedSimCity simulation. This applet allows users to select simulation
 * parameters, start the simulation, and view simulation progress. It provides
 * options for choosing the simulation mode, city name, and other
 * simulation-specific settings. Users can also enable specific
 * origin-destination (OD) testing and access other advanced options.
 */
public class PedSimCityNightApplet extends PedSimCityApplet {

	private static final long serialVersionUID = 1L;

	private NightParametersPanel nightPanel;

	public PedSimCityNightApplet() {
		super();
		Pars.isNight = true;
		
		routeParsButton.setVisible(true);
		routeParsButton.setLabel("Night Parameters");
		
		// Remove existing action listeners from superclass
		for (java.awt.event.ActionListener al : routeParsButton.getActionListeners()) {
			routeParsButton.removeActionListener(al);
		}
		
		routeParsButton.addActionListener(e -> openNightPanel());
	}

	private void openNightPanel() {
		if (nightPanel == null) {
			nightPanel = new NightParametersPanel();
		}
		nightPanel.setVisible(true);
	}

	public static void main(String[] args) throws Exception {
		boolean headless = false;
		boolean forceWebsite = false;
		boolean forceGui = false;

		for (String arg : args) {
			if ("--headless".equals(arg) || arg.startsWith("--headless="))
				headless = true;
			if ("--website".equals(arg))
				forceWebsite = true;
			if ("--gui".equals(arg))
				forceGui = true;
		}

		if (headless) {
			LoggerUtil.getLogger().info("[SERVER] Running headless night simulation...");
			ParameterManager.initFromArgsForServer(args);
			Pars.isNight = true;

			ScenarioConfig scenarioConfig = new ScenarioConfig(StringEnum.Vulnerable.values(),
					StringEnum.TimeOfDay.values());
			Engine.StateFactory stateFactory = PedSimCityNight::new;

			Engine engine = new Engine(stateFactory);
			engine.runJobs(scenarioConfig, Pars.parallel);
			return;
		}

		// Interactive selection if no flags provided
		int choice = 0;
		if (forceWebsite)
			choice = 2;
		else if (forceGui)
			choice = 1;
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
				choice = 1; // Default to GUI
			}
		}

		if (choice == 2) {
			LoggerUtil.getLogger().info("[STARTUP] Starting REST API for External Dashboard...");
			// Pre-load default GIS data so the dashboard can show the map immediately
			try {
				pedsim.core.engine.PedSimCity.clearStaticData();
				pedsim.core.parameters.Pars.setSimulationParameters();
				new pedsim.core.engine.Import().importFiles();
				pedsim.core.engine.SimulationStateStore.getInstance().setRoadsGeoJson(
						pedsim.core.website.GeoJsonExporter.exportRoads(pedsim.core.engine.PedSimCity.roads));
			} catch (Exception e) {
				LoggerUtil.getLogger().warning("Could not pre-load default roads: " + e.getMessage());
			}

			SimulationRestApi.setOnStart((params) -> {
				try {
					// Apply parameters if provided
					if (params.containsKey("cityName"))
						Pars.cityName = (String) params.get("cityName");
					if (params.containsKey("days"))
						Pars.durationDays = Integer.parseInt(params.get("days").toString());
					if (params.containsKey("actualPopulation"))
						Pars.population = Integer.parseInt(params.get("actualPopulation").toString());
					if (params.containsKey("percentage"))
						Pars.percentagePopulationAgent = Double.parseDouble(params.get("percentage").toString());
					if (params.containsKey("jobs")) {
						Pars.jobs = Integer.parseInt(params.get("jobs").toString());
						Pars.parallel = (Pars.jobs > 1);
					}

					Pars.setSimulationParameters(); // Recalculate derived parameters

					ScenarioConfig config = new ScenarioConfig(StringEnum.Vulnerable.values(),
							StringEnum.TimeOfDay.values());
					new Engine(PedSimCityNight::new).runJobs(config, Pars.jobs > 1); // Use boolean for parallel
				} catch (Exception e) {
					LoggerUtil.getLogger().severe("Simulation failed: " + e.getMessage());
				}
			});

			SimulationRestApi.start(8081);
		} else {
			LoggerUtil.getLogger().info("[STARTUP] Launching Standard GUI...");

			// Start REST API in background for external visualization
			SimulationRestApi.start(8081);

			// Auto-open the browser
			try {
				if (Desktop.isDesktopSupported()) {
					Desktop.getDesktop().browse(new URI("http://localhost:8080"));
				}
			} catch (Exception e) {
				LoggerUtil.getLogger().warning("Could not open browser: " + e.getMessage());
			}
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
