package pedSim.applet;

import java.util.List;
import java.util.stream.IntStream;

import javax.swing.JFrame;
import javax.swing.JScrollPane;

import pedSim.agents.Agent;
import pedSim.engine.PedSimCity;
import pedSim.parameters.Pars;
import sim.engine.SimState;

public class Run {

//	static ArrayList<FlowHandler> flowHandlers = new ArrayList<>();

	public static void ParallelRun() {

		IntStream.range(0, TestPars.jobs).parallel().forEach(job -> {
			final SimState state = new PedSimCity(System.currentTimeMillis(), job);
			state.start();
			List<Agent> agentList = ((PedSimCity) state).getAgentsList();
			while (state.schedule.step(state)) {
				PedSimCityImageApplet.remainingTripsCount = agentList.parallelStream()
						.mapToInt(agent -> agent.OD.size() - agent.tripsDone).sum() * TestPars.jobs;
				PedSimCityImageApplet.updateRemainingTripsLabel(true);
			}
			flowHandlers.add(((PedSimCity) state).flowHandler);
		});
	}

	public static void VisRun() {
		for (int job = 0; job < TestPars.jobs; job++) {
			PedSimCityApplet.jobLabel.setText("Executing Job Nr: " + job);
			final SimState state = new PedSimCity(System.currentTimeMillis(), job);
			state.start();
			List<Agent> agentList = ((PedSimCity) state).getAgentsList();
			setGeoVis(agentList);

			((PedSimCity) state).startSchedule();

			// Create and start a rendering timer for smooth visual updates
//			Timer renderTimer = new Timer(16, e -> {
//				PedSimCityApplet.panel.repaint();
//			});
//			renderTimer.start();

			while (state.schedule.step(state)) {
				PedSimCityApplet.panel.repaint();
				PedSimCityApplet.remainingTripsCount = agentList.parallelStream()
						.mapToInt(agent -> agent.OD.size() - agent.tripsDone).sum();
				PedSimCityApplet.updateRemainingTripsLabel(false);
			}
//			renderTimer.stop();
			flowHandlers.add(((PedSimCity) state).flowHandler);
		}
	}

//	public void RunVis() {
//
//		for (int job = 0; job < Parameters.jobs; job++) {
//			PedSimCityApplet.jobLabel.setText("Executing Job Nr: " + job);
//			final SimState state = new PedSimCity(System.currentTimeMillis(), job);
//			state.start();
//			List<Agent> agentList = ((PedSimCity) state).getAgentsList();
//			setGeoVis(agentList);
//
//			((PedSimCity) state).startSchedule();
//
//			int substeps = 10; // Number of substeps per step
//
//			while (state.schedule.step(state)) {
//				System.out.println("steps: " + state.schedule.getSteps());
//
//				// Perform substeps
//				for (int substep = 0; substep < substeps; substep++) {
//					// Update agents and simulation for the current substep
//					agentList.forEach(agent -> agent.updateForSubstep(substep, substeps));
//
//					// Repaint the panel to reflect the current substep
//					panel.repaint();
//
//					// Optional: Add a small delay for smoother visualization (e.g., 50ms)
//					try {
//						Thread.sleep(50);
//					} catch (InterruptedException e) {
//						Thread.currentThread().interrupt();
//						break;
//					}
//				}
//
//				// Compute the remaining trips count after the main step
//				remainingTripsCount = agentList.parallelStream().mapToInt(agent -> agent.OD.size() - agent.tripsDone)
//						.sum();
//				updateRemainingTripsLabel(false);
//			}
//			flowHandlers.add(((PedSimCity) state).flowHandler);
//		}
//	}

	static void setGeoVis(List<Agent> agentList) {

		PedSimCityApplet.panel = new Display(agentList);
		PedSimCityApplet.frame.add(PedSimCityApplet.panel);
		PedSimCityApplet.frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
		PedSimCityApplet.frame.setVisible(true);
		JScrollPane scrollPane = new JScrollPane(PedSimCityApplet.panel);
		PedSimCityApplet.frame.add(scrollPane);

	}

}
