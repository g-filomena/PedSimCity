package pedsim.core.engine;

import java.awt.Desktop;
import java.io.File;
import java.util.logging.Logger;
import java.util.stream.IntStream;

import pedsim.core.parameters.Pars;
import pedsim.core.parameters.TimePars;
import pedsim.core.utilities.LoggerUtil;
import pedsim.core.website.GeoJsonExporter;
import sim.util.geo.Utilities;

public class Engine {

	private static final Logger logger = LoggerUtil.getLogger();

	protected final StateFactory stateFactory;
	protected final long baseSeed;

	/** Records a compact agent-position snapshot at every simulation step. */
	private TrajectoryRecorder trajectoryRecorder;

	@FunctionalInterface
	public interface StateFactory {
		PedSimCity create(long seed, int job, ScenarioConfig scenarioConfig);
	}

	public Engine(StateFactory stateFactory) {
		this(stateFactory, System.currentTimeMillis());
	}

	public Engine(StateFactory stateFactory, long baseSeed) {
		this.stateFactory = stateFactory;
		this.baseSeed = baseSeed;
	}

	public synchronized void runJobs(ScenarioConfig scenarioConfig, boolean parallel) throws Exception {
		if (SimulationStateStore.getInstance().running) {
			logger.warning("Simulation is already running! Ignoring new run request.");
			return;
		}

		try {
			SimulationStateStore.getInstance().running = true;
			SimulationStateStore.getInstance().finished = false;
			SimulationStateStore.getInstance().stopRequested = false;

			clearStaticData();
			Pars.setSimulationParameters();

			Import importer = new Import();
			importer.importFiles();

			// Export road network as GeoJSON once so the browser map can draw it
			SimulationStateStore.getInstance().setRoadsGeoJson(GeoJsonExporter.exportRoads(PedSimCity.roads));

			Environment.prepare();
			logger.info("Environment prepared. About to start simulation");

			if (parallel) {
				IntStream.range(0, Pars.jobs).parallel().forEach(jobNr -> {
					try {
						Engine engine = createWorkerEngine(); // one engine per worker
						logger.info("Executing Job nr.: " + jobNr);
						engine.executeJob(jobNr, scenarioConfig);
					} catch (Exception e) {
						throw new RuntimeException("Error executing parallel job " + jobNr, e);
					}
				});
			} else {
				for (int jobNr = 0; jobNr < Pars.jobs; jobNr++) {
					logger.info("Executing Job nr.: " + jobNr);
					executeJob(jobNr, scenarioConfig);
				}
			}

		} finally {
			SimulationStateStore.getInstance().running = false;
			SimulationStateStore.getInstance().finished = true;
		}
	}

	
    protected void clearStaticData() {
        PedSimCity.clearStaticData();
    }

    protected Engine createWorkerEngine() {
        return new Engine(stateFactory, baseSeed);
    }

protected void prepareToRun() throws Exception {
		Pars.setSimulationParameters();

		Import importer = new Import();
		importer.importFiles();

		Environment.prepare();
		logger.info("Environment prepared. About to start simulation");
	}

	public void executeJob(int job, ScenarioConfig scenarioConfig) throws Exception {
		int currentDay = 0;

		long seed = seedForJob(job);
		PedSimCity state = stateFactory.create(seed, job, scenarioConfig);

		// Clear and initialise the trajectory recorders for this job
		TripRouteRecorder.clear();
		trajectoryRecorder = new TrajectoryRecorder(state);

		state.start();

		onJobStarted(job, state, scenarioConfig);

		double kmCurrentDay = calculateMetersCurrentDay();
		logger.info("---------- Beginning day Nr " + (currentDay + 1));
		AgentReleaseManager currentDayReleaseManager = new AgentReleaseManager(state, kmCurrentDay, currentDay + 1);

		double nextAgentRelease = 1.0;

		while (continueSimulation(state)) {
			onJobStep(job, state, scenarioConfig);

			double steps = state.schedule.getSteps();

			// Check whether the Javelit dashboard has requested a stop
			if (SimulationStateStore.getInstance().stopRequested) {
				logger.info("Stop requested by dashboard - ending simulation.");
				break;
			}

			// Push step-level statistics to the browser dashboard
			String simTime = TimePars.getTime(steps).toLocalTime().toString();
			SimulationStateStore.getInstance().updateStep((int) steps, simTime, state.agentsWalking.size(),
					state.agentsAtHome.size(), 0);

			// Record agent positions for the HTML dashboard
			trajectoryRecorder.record((long) steps);

			if (isNextDay(steps, currentDay)) {
				state.flowHandler.updateCognitiveMapsData(null);
				state.flowHandler.exportFlowsData(currentDay + 1);
				state.flowHandler.exportCognitiveMapsData(currentDay + 1);
				currentDay++;

				if (currentDay % 6 == 0) {
					handleEndWeek(state, job, scenarioConfig);
				} else {
					kmCurrentDay = calculateMetersCurrentDay();
					logger.info("---------- Beginning day Nr " + (currentDay + 1));
					currentDayReleaseManager = new AgentReleaseManager(state, kmCurrentDay, currentDay + 1);
				}
			}

			if (steps >= nextAgentRelease) {
				currentDayReleaseManager.releaseAgents(steps);
				nextAgentRelease += TimePars.releaseAgentsEverySteps;
			}

		}

		state.flowHandler.updateCognitiveMapsData(null);
		state.flowHandler.exportFlowsData(currentDay + 1);
		state.flowHandler.exportCognitiveMapsData(currentDay + 1);

		onJobFinished(job, state, scenarioConfig);
		state.finish();

		TripRouteRecorder.saveToFile("test_trips.csv");
		TripDiagnostic.save("trip_diagnostic.csv");


		// Generate the self-contained HTML dashboard and open it in the browser
		generateAndOpenHtmlDashboard(job, state, currentDay);
	}

	private void generateAndOpenHtmlDashboard(int job, PedSimCity state, int currentDay) {
		try {
			logger.info("[Engine] Compiling HTML dashboard for job " + job + "…");

			String htmlPath = HtmlExporter.export(
				currentDay + 1, // day (1-based)
				job,
				TripRouteRecorder.getRecords(),
				state.flowHandler.volumesMap
			);

			if (htmlPath != null && Desktop.isDesktopSupported()) {
				Desktop.getDesktop().browse(new File(htmlPath).toURI());
				logger.info("[Engine] Opened dashboard in browser: " + htmlPath);
			}

		} catch (Exception e) {
			logger.warning("[Engine] Could not open HTML dashboard: " + e.getMessage());
		}
	}

	protected long seedForJob(int job) {
		return baseSeed + job;
	}

	protected boolean continueSimulation(PedSimCity state) {
		return state.schedule.step(state) && (state.schedule.getSteps() <= TimePars.simulationDurationInSteps);
	}

	protected void onJobStarted(int job, PedSimCity state, ScenarioConfig scenarioConfig) {
		// no-op
	}

	protected void onJobStep(int job, PedSimCity state, ScenarioConfig scenarioConfig) {
		// no-op
	}

	protected void onJobFinished(int job, PedSimCity state, ScenarioConfig scenarioConfig) {
		// no-op
	}

	protected void handleEndWeek(PedSimCity state, int job, ScenarioConfig scenarioConfig) {
		// no-op
	}

	protected boolean isNextDay(double steps, int currentDay) {
		return getDays(steps) > currentDay && (currentDay + 1 < TimePars.numberOfDays);
	}

	public static long getDays(double totalSteps) {
		long totalMinutes = (long) (totalSteps * (TimePars.STEP_DURATION / 60));
		return totalMinutes / (24 * 60);
	}

	protected double calculateMetersCurrentDay() {
		return Pars.metersPerDay * Utilities.fromDistribution(1.0, 0.10, null);
	}
}
