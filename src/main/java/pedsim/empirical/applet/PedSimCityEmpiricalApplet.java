package pedsim.empirical.applet;

import java.awt.Button;
import java.awt.Checkbox;
import java.awt.Choice;
import java.awt.Color;
import java.awt.Frame;
import java.awt.Label;
import java.awt.TextField;
import java.awt.event.WindowAdapter;
import java.awt.event.WindowEvent;

import pedsim.empirical.parameters.EmpiricalPars;

/** Minimal GUI launcher for the empirical ABM. */
public class PedSimCityEmpiricalApplet extends Frame {

	private static final long serialVersionUID = 1L;

	private final Choice cityChoice = new Choice();
	private final TextField jobsField = new TextField(Integer.toString(EmpiricalPars.defaultJobs));
	private final TextField agentsField = new TextField(Integer.toString(EmpiricalPars.defaultNumAgents));
	private final TextField tripsField = new TextField(Integer.toString(EmpiricalPars.numberTripsPerAgent));
	private final Checkbox dmaCheckbox = new Checkbox("Use DMA OD generation", true);

	private final Label jobLabel = new Label("Idle");
	private final Label remainingTripsLabel = new Label("Trips left: -");
	private final Label statusLabel = new Label("Ready");

	private Thread simulationThread;

	public PedSimCityEmpiricalApplet() {
		super("PedSimCity Empirical ABM");

		setLayout(null);
		buildForm();

		setSize(430, 340);
		setVisible(true);

		addWindowListener(new WindowAdapter() {
			@Override
			public void windowClosing(WindowEvent event) {
				dispose();
			}
		});
	}

	private void buildForm() {
		Label cityLabel = new Label("City:");
		cityLabel.setBounds(10, 40, 120, 20);
		add(cityLabel);

		cityChoice.setBounds(150, 40, 220, 20);
		cityChoice.add("Muenster");
		add(cityChoice);

		Label jobsLabel = new Label("Jobs:");
		jobsLabel.setBounds(10, 75, 120, 20);
		add(jobsLabel);

		jobsField.setBounds(150, 75, 80, 20);
		add(jobsField);

		Label agentsLabel = new Label("Agents:");
		agentsLabel.setBounds(10, 110, 120, 20);
		add(agentsLabel);

		agentsField.setBounds(150, 110, 80, 20);
		add(agentsField);

		Label tripsLabel = new Label("Trips per agent:");
		tripsLabel.setBounds(10, 145, 120, 20);
		add(tripsLabel);

		tripsField.setBounds(150, 145, 80, 20);
		add(tripsField);

		dmaCheckbox.setBounds(10, 180, 250, 25);
		add(dmaCheckbox);

		Button startButton = new Button("Run Empirical ABM");
		startButton.setBounds(10, 220, 150, 45);
		startButton.setBackground(new Color(0, 200, 0));
		startButton.addActionListener(event -> startSimulation());
		add(startButton);

		Button stopButton = new Button("Stop");
		stopButton.setBounds(180, 220, 80, 45);
		stopButton.setBackground(Color.PINK);
		stopButton.addActionListener(event -> stopSimulation());
		add(stopButton);

		jobLabel.setBounds(10, 275, 250, 20);
		add(jobLabel);

		remainingTripsLabel.setBounds(10, 295, 250, 20);
		add(remainingTripsLabel);

		statusLabel.setBounds(10, 315, 350, 20);
		add(statusLabel);
	}

	private void startSimulation() {
		if (simulationThread != null && simulationThread.isAlive()) {
			statusLabel.setText("Simulation already running.");
			return;
		}

		statusLabel.setText("Simulation running...");

		simulationThread = new Thread(() -> Run.visualRun(this), "empirical-abm-runner");
		simulationThread.start();
	}

	private void stopSimulation() {
		if (simulationThread != null && simulationThread.isAlive()) {
			simulationThread.interrupt();
			statusLabel.setText("Stop requested. Close windows if needed.");
		}
	}

	public String getSelectedCityName() {
		return cityChoice.getSelectedItem();
	}

	public int getJobs() {
		return parsePositiveInt(jobsField.getText(), EmpiricalPars.defaultJobs);
	}

	public int getNumAgents() {
		return parsePositiveInt(agentsField.getText(), EmpiricalPars.defaultNumAgents);
	}

	public int getTripsPerAgent() {
		return parsePositiveInt(tripsField.getText(), EmpiricalPars.numberTripsPerAgent);
	}

	public boolean isUsingDMA() {
		return dmaCheckbox.getState();
	}

	public void setJobLabel(String text) {
		jobLabel.setText(text);
	}

	public void setRemainingTripsLabel(String text) {
		remainingTripsLabel.setText(text);
	}

	public void markSimulationEnded() {
		statusLabel.setText("Simulation ended.");
	}

	public void markSimulationFailed(String message) {
		statusLabel.setText("Simulation failed: " + message);
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
		new PedSimCityEmpiricalApplet();
	}
}