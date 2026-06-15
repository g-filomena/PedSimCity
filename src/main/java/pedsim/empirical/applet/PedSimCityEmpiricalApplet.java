package pedsim.empirical.applet;

import java.awt.Checkbox;
import java.awt.Label;
import java.awt.TextField;
import java.awt.event.WindowAdapter;
import java.awt.event.WindowEvent;

import pedsim.core.applet.PedSimCityApplet;
import pedsim.core.applet.ServerProjectConfig;
import pedsim.core.engine.Engine;
import pedsim.core.engine.ScenarioConfig;
import pedsim.core.parameters.ParameterManager;
import pedsim.core.parameters.Pars;
import pedsim.core.utilities.LoggerUtil;
import pedsim.empirical.agent.EmpiricalGroup;
import pedsim.empirical.engine.EmpiricalEngine;
import pedsim.empirical.engine.PedSimCityEmpirical;
import pedsim.empirical.parameters.EmpiricalPars;

public class PedSimCityEmpiricalApplet extends PedSimCityApplet {

	private static final long serialVersionUID = 1L;

	private final TextField agentsField = new TextField(Integer.toString(EmpiricalPars.defaultNumAgents));
	private final TextField tripsField = new TextField(Integer.toString(EmpiricalPars.numberTripsPerAgent));
	private final Checkbox dmaCheckbox = new Checkbox("Use DMA OD generation", true);

	private static Label jobLabel;
	private static Label remainingTripsLabel;

	public PedSimCityEmpiricalApplet() {
		super();
		addEmpiricalControls();
		setSize(500, 580);
		validate();
	}

	private void addEmpiricalControls() {
		Label agentsLabel = new Label("Agents:");
		agentsLabel.setBounds(10, 215, 120, 20);
		add(agentsLabel);
		agentsField.setBounds(190, 215, 100, 20);
		add(agentsField);

		Label tripsLabel = new Label("Trips per agent:");
		tripsLabel.setBounds(10, 245, 140, 20);
		add(tripsLabel);
		tripsField.setBounds(190, 245, 100, 20);
		add(tripsField);

		dmaCheckbox.setBounds(10, 275, 250, 25);
		add(dmaCheckbox);

		jobLabel = new Label("Job: -");
		jobLabel.setBounds(10, 440, 230, 20);
		jobLabel.setVisible(false);
		add(jobLabel);

		remainingTripsLabel = new Label("Trips left: -");
		remainingTripsLabel.setBounds(10, 465, 230, 20);
		remainingTripsLabel.setVisible(false);
		add(remainingTripsLabel);
	}

	@Override
	protected String getAppletTitle() {
		return "PedSimCity Empirical ABM";
	}

	@Override
	protected void updateCityNameOptions() {
		if (cityName == null) return;
		cityName.removeAll();
		cityName.add("Muenster");
		cityName.validate();
	}

	@Override
	protected Engine.StateFactory buildStateFactory() {
		return PedSimCityEmpirical::new;
	}

	@Override
	protected Engine buildEngine() {
		return new EmpiricalEngine(buildStateFactory());
	}

	@Override
	protected ScenarioConfig buildScenarioConfig() {
		return new ScenarioConfig(EmpiricalGroup.values(), null);
	}

	@Override
	protected void launchSimulation(boolean runInParallel) {
		setRunningOnServer(false);

		Thread existingThread = getSimulationThread();
		if (existingThread != null && existingThread.isAlive()) {
			LoggerUtil.getLogger().warning("Empirical simulation is already running.");
			return;
		}

		ParameterManager.collectParameters(this);
		applyEmpiricalParameters(runInParallel);

		final ScenarioConfig scenarioConfig = buildScenarioConfig();
		final Engine engine = buildEngine();

		initialiseProgressLabels();

		Thread thread = new Thread(() -> {
			try {
				engine.runJobs(scenarioConfig, runInParallel);
			} catch (Exception exception) {
				LoggerUtil.getLogger().severe("Empirical simulation failed: " + exception.getMessage());
				exception.printStackTrace();
			}
		}, "empirical-simulation");

		setSimulationThread(thread);
		thread.start();
	}

	private void applyEmpiricalParameters(boolean runInParallel) {
		Pars.cityName = getCityName();
		EmpiricalPars.numAgents = parsePositiveInt(agentsField.getText(), EmpiricalPars.defaultNumAgents);
		EmpiricalPars.numberTripsPerAgent = parsePositiveInt(tripsField.getText(), EmpiricalPars.numberTripsPerAgent);
		EmpiricalPars.usingDMA = dmaCheckbox.getState();
		Pars.parallel = runInParallel;
	}

	private void initialiseProgressLabels() {
		if (jobLabel != null) {
			jobLabel.setText("Job: running...");
			jobLabel.setVisible(true);
		}
		if (remainingTripsLabel != null) {
			remainingTripsLabel.setText("Trips left: -");
			remainingTripsLabel.setVisible(true);
		}
	}

	public static void setJobLabel(String text) {
		if (jobLabel != null) jobLabel.setText(text);
	}

	public static void setRemainingTripsLabel(String text) {
		if (remainingTripsLabel != null) remainingTripsLabel.setText(text);
	}

	@Override
	protected ServerProjectConfig buildServerProjectConfig() {
		return new ServerProjectConfig("/mnt/home/gabriele/PedSimCityEmpirical",
				"pedsim.empirical.applet.PedSimCityEmpiricalApplet", "bin:lib/*:src/main/resources");
	}

	private static int parsePositiveInt(String value, int fallback) {
		try {
			int parsed = Integer.parseInt(value.trim());
			return parsed > 0 ? parsed : fallback;
		} catch (Exception exception) {
			return fallback;
		}
	}

	public static void main(String[] args) {
		PedSimCityEmpiricalApplet applet = new PedSimCityEmpiricalApplet();
		applet.addWindowListener(new WindowAdapter() {
			@Override
			public void windowClosing(WindowEvent event) {
				applet.dispose();
			}
		});
	}
}
