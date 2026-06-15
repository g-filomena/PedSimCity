package pedsim.cityimage.engine;

import java.awt.EventQueue;
import java.awt.GraphicsEnvironment;
import java.util.Set;
import java.util.concurrent.ConcurrentHashMap;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.logging.Logger;

import pedsim.cityimage.applet.PedSimCityImageApplet;
import pedsim.cityimage.parameters.TestPars;
import pedsim.core.agents.Agent;
import pedsim.core.engine.Engine;
import pedsim.core.engine.Import;
import pedsim.core.engine.PedSimCity;
import pedsim.core.engine.ScenarioConfig;
import pedsim.core.engine.SimulationStateStore;
import pedsim.core.utilities.LoggerUtil;

/** Engine specialised for the city-image testing module. */
public class CityImageEngine extends Engine {

	private static final Logger LOGGER = LoggerUtil.getLogger();

	private final ConcurrentHashMap<Integer, Integer> remainingTripsByJob = new ConcurrentHashMap<>();
	private final AtomicInteger totalRemainingTrips = new AtomicInteger(0);

	public CityImageEngine(StateFactory stateFactory) {
		super(stateFactory);
	}

	public CityImageEngine(StateFactory stateFactory, long baseSeed) {
		super(stateFactory, baseSeed);
	}

	@Override
	protected Import createImporter() {
		return new CityImageImport();
	}

	@Override
	protected void clearStaticData() {
		super.clearStaticData();
		TestPars.distances.clear();
		PedSimCityImage.indexedEdgeCache.clear();
	}

	@Override
	protected Engine createWorkerEngine() {
		return new CityImageEngine(stateFactory, baseSeed);
	}

	@Override
	public void executeJob(int job, ScenarioConfig scenarioConfig) throws Exception {
		long seed = seedForJob(job);
		PedSimCityImage state = (PedSimCityImage) stateFactory.create(seed, job, scenarioConfig);

		state.start();

		onJobStarted(job, state, scenarioConfig);

		Set<?> agentList = state.getAgentsList();

		while (state.schedule.step(state)) {
			updateRemainingTrips(job, agentList);
			onJobStep(job, state, scenarioConfig);

			if (SimulationStateStore.getInstance().stopRequested) {
				LOGGER.info("Stop requested - ending CityImage job.");
				break;
			}
		}

		updateRemainingTrips(job, agentList);
		onJobFinished(job, state, scenarioConfig);

		state.finish();
	}

	@Override
	protected void onJobStarted(int job, PedSimCity state, ScenarioConfig scenarioConfig) {
		remainingTripsByJob.put(job, 0);
	}

	@Override
	protected void onJobFinished(int job, PedSimCity state, ScenarioConfig scenarioConfig) {
		remainingTripsByJob.remove(job);
	}

	private void updateRemainingTrips(int job, Set<?> agentList) {
		int currentRemaining = 0;

		for (Object object : agentList) {
			if (object instanceof Agent agent) {
				currentRemaining += Math.max(0, agent.OD.size() - agent.getTripsDone());
			}
		}

		Integer previous = remainingTripsByJob.put(job, currentRemaining);
		int previousValue = previous == null ? 0 : previous;

		int total = totalRemainingTrips.addAndGet(currentRemaining - previousValue);

		if (GraphicsEnvironment.isHeadless()) {
			return;
		}

		EventQueue.invokeLater(() -> {
			PedSimCityImageApplet.setRemainingTripsCount(total);
			PedSimCityImageApplet.updateRemainingTripsLabel(true);
		});
	}
}
