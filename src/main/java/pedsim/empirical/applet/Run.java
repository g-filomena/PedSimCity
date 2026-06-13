package pedsim.empirical.applet;

import javax.swing.JFrame;
import javax.swing.JScrollPane;

import pedsim.core.engine.Environment;
import pedsim.core.engine.PedSimCity;
import pedsim.core.engine.ScenarioConfig;
import pedsim.core.parameters.Pars;
import pedsim.empirical.agent.EmpiricalGroup;
import pedsim.empirical.engine.EmpiricalImport;
import pedsim.empirical.engine.PedSimCityEmpirical;
import pedsim.empirical.parameters.EmpiricalPars;
import sim.engine.SimState;

/** Runner used by the empirical applet. */
public final class Run {

	private Run() {
	}

	public static void visualRun(PedSimCityEmpiricalApplet applet) {
		try {
			configureFromApplet(applet);

			PedSimCity.clearStaticData();

			EmpiricalImport importer = new EmpiricalImport();
			importer.importFiles();

			Environment.prepare();

			ScenarioConfig scenarioConfig = new ScenarioConfig(EmpiricalGroup.values(), null);

			for (int job = 0; job < Pars.jobs; job++) {
				applet.setJobLabel("Executing Job Nr: " + job);

				SimState state = new PedSimCityEmpirical(System.currentTimeMillis(), job, scenarioConfig);
				state.start();

				PedSimCityEmpirical empiricalState = (PedSimCityEmpirical) state;

				EmpiricalDisplay display = new EmpiricalDisplay(empiricalState.getAgentsList());
				showDisplay(display);

				while (state.schedule.step(state)) {
					display.setAgents(empiricalState.getAgentsList());
					display.repaint();

					int remainingTrips = empiricalState.getAgentsList().parallelStream()
							.mapToInt(agent -> agent.OD.size() - agent.getTripsDone()).sum();

					applet.setRemainingTripsLabel("Trips left: " + remainingTrips);
				}

				state.finish();
			}

			applet.markSimulationEnded();

		} catch (Exception exception) {
			exception.printStackTrace();
			applet.markSimulationFailed(exception.getMessage());
		}
	}

	private static void configureFromApplet(PedSimCityEmpiricalApplet applet) {
		EmpiricalPars.applyDefaults();

		Pars.cityName = applet.getSelectedCityName();
		Pars.numAgents = applet.getNumAgents();
		Pars.jobs = applet.getJobs();

		EmpiricalPars.numberTripsPerAgent = applet.getTripsPerAgent();
		EmpiricalPars.usingDMA = applet.isUsingDMA();
	}

	private static void showDisplay(EmpiricalDisplay display) {
		JFrame frame = new JFrame("PedSimCity Empirical Agents");
		frame.setDefaultCloseOperation(JFrame.DISPOSE_ON_CLOSE);
		frame.add(new JScrollPane(display));
		frame.pack();
		frame.setVisible(true);
	}
}