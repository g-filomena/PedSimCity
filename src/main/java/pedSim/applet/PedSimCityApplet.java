package pedSim.applet;

import java.awt.Button;
import java.awt.Checkbox;
import java.awt.Choice;
import java.awt.Color;
import java.awt.Frame;
import java.awt.Label;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.awt.event.ItemEvent;
import java.awt.event.ItemListener;
import java.awt.event.WindowAdapter;
import java.awt.event.WindowEvent;
import java.util.ArrayList;
import java.util.List;
import java.util.logging.Logger;
import java.util.stream.IntStream;

import pedSim.agents.Agent;
import pedSim.engine.Environment;
import pedSim.engine.FlowHandler;
import pedSim.engine.Import;
import pedSim.engine.Parameters;
import pedSim.engine.PedSimCity;
import sim.engine.SimState;

/**
 * A graphical user interface (GUI) applet for configuring and running the
 * PedSimCity simulation. This applet allows users to select simulation
 * parameters, start the simulation, and view simulation progress. It provides
 * options for choosing the simulation mode, city name, and other
 * simulation-specific settings. Users can also enable specific
 * origin-destination (OD) testing and access other advanced options.
 */
public class PedSimCityApplet extends Frame implements ItemListener {

	private static final long serialVersionUID = 1L;
	private Choice cityName;
	private Choice modeChoice;
	private Button startButton;
	private Button endButton;
	private Button startButtonParallel;
	private Checkbox specificODcheckBox;
	private Checkbox verboseCheckBox;
	private Label jobLabel;
	private Label remainingTripsLabel;
	private Label jobsLabel;
	private Label remainingTripsLabelParallel;
	private int remainingTripsCount;
	private static final Logger LOGGER = Logger.getLogger(Import.class.getName());
	private Thread simulationThread;

	/**
	 * Constructs a new instance of the `PedSimCityApplet` class, creating a
	 * graphical user interface (GUI) applet for configuring and running the
	 * PedSimCity simulation. Initialises and arranges various GUI components,
	 * including mode selection, city selection, and simulation control buttons.
	 */
	public PedSimCityApplet() {
		super("PedSimCity Applet");
		setLayout(null);

		Label modeLabel = new Label("Simulation Mode:");
		modeLabel.setBounds(10, 40, 100, 20);
		add(modeLabel);

		modeChoice = new Choice();
		modeChoice.setBounds(140, 40, 230, 20);
		modeChoice.add("Testing Landmarks");
		modeChoice.add("Testing Urban Subdivisions");
		modeChoice.add("Testing Specific Route Choice Models");
		modeChoice.add("Empirical ABM");
		modeChoice.addItemListener(this); // Add ItemListener to handle changes in modeChoice
		add(modeChoice);

		Label cityNameLabel = new Label("City Name:");
		cityNameLabel.setBounds(10, 70, 80, 20);
		add(cityNameLabel);

		cityName = new Choice();
		cityName.setBounds(140, 70, 150, 20);
		updateCityNameOptions(); // Set initial options based on the default selection of modeChoice
		add(cityName);

		specificODcheckBox = new Checkbox("Testing Specific ODs");
		specificODcheckBox.setEnabled(true);
		specificODcheckBox.setBounds(10, 100, 300, 40);
		specificODcheckBox.addItemListener(this);
		add(specificODcheckBox);

		Button otherOptionsButton = new Button("Other Options");
		otherOptionsButton.setBounds(10, 150, 150, 30);
		otherOptionsButton.addActionListener(new ActionListener() {
			public void actionPerformed(ActionEvent e) {
				openParametersPanel();
			}
		});
		add(otherOptionsButton);

		Button routeChoiceButton = new Button("Choose Route-Choices");
		routeChoiceButton.setEnabled(false);
		routeChoiceButton.setBounds(10, 200, 150, 30);
		routeChoiceButton.addActionListener(new ActionListener() {
			public void actionPerformed(ActionEvent e) {
				openTestPanel();
			}
		});
		add(routeChoiceButton);

		modeChoice.addItemListener(new ItemListener() {
			public void itemStateChanged(ItemEvent e) {
				String selectedMode = modeChoice.getSelectedItem();
				// Enable the button only if the selected mode is "Testing Specific Route
				// Choice// Models"
				routeChoiceButton.setEnabled(selectedMode.equals("Testing Specific Route Choice Models"));
				specificODcheckBox.setEnabled(!selectedMode.equals("Empirical ABM"));
			}
		});

		verboseCheckBox = new Checkbox("Verbose y/n");
		verboseCheckBox.setEnabled(true);
		verboseCheckBox.setBounds(10, 230, 300, 40);
		verboseCheckBox.addItemListener(this);
		add(verboseCheckBox);

		Color color = new Color(0, 220, 0);
		startButton = new Button("Run Simulation");
		startButton.setBounds(10, 280, 120, 50);
		startButton.setBackground(color);
		add(startButton);

		endButton = new Button("End Simulation");
		endButton.setBackground(Color.PINK);

		startButton.addActionListener(new ActionListener() {
			public void actionPerformed(ActionEvent e) {
				endButton.setBounds(10, 280, 120, 50);
				startButtonParallel.setVisible(false);
				add(endButton);
				startButton.setVisible(false);
				simulationThread = new Thread(() -> startSimulation(false));
				simulationThread.start();
			}
		});

		endButton.addActionListener(new ActionListener() {
			public void actionPerformed(ActionEvent e) {
				if (simulationThread != null && simulationThread.isAlive()) {
					System.exit(0);
				}
			}
		});

		jobLabel = new Label("Executing Job Nr:");
		jobLabel.setBounds(140, 280, 120, 20);
		jobLabel.setVisible(false);

		add(jobLabel);

		remainingTripsLabel = new Label("Trips left (jobs avg):");
		remainingTripsLabel.setBounds(140, 310, 170, 20);
		remainingTripsLabel.setVisible(false);
		add(remainingTripsLabel);

		// Parallel
		startButtonParallel = new Button("Run in Parallel");
		startButtonParallel.setBounds(10, 350, 120, 50);
		startButtonParallel.setBackground(color);
		add(startButtonParallel);

		startButtonParallel.addActionListener(new ActionListener() {
			public void actionPerformed(ActionEvent e) {
				endButton.setBounds(10, 350, 120, 50);
				startButton.setVisible(false);
				add(endButton);
				startButtonParallel.setVisible(false);
				simulationThread = new Thread(() -> startSimulation(true));
				simulationThread.start();
			}
		});

		jobsLabel = new Label("Parallelising ? Jobs");
		jobsLabel.setBounds(140, 350, 120, 20);
		jobsLabel.setVisible(false);
		add(jobsLabel);

		remainingTripsLabelParallel = new Label("Trips left (jobs avg):");
		remainingTripsLabelParallel.setBounds(140, 380, 170, 20);
		remainingTripsLabelParallel.setVisible(false);
		add(remainingTripsLabelParallel);

		setSize(450, 450);
		setVisible(true);

	}

	/**
	 * Handles changes in the state of the SpecificODcheckbox by either opening or
	 * closing the SpecificODpanel.
	 *
	 * @param e The ItemEvent that triggered the change in the state of the
	 *          SpecificODcheckbox.
	 */
	public void ODpanelStateChanged(ItemEvent e) {
		SpecificODpanel specificODpanel = new SpecificODpanel();
		if (specificODcheckBox.getState())
			specificODpanel.handleSpecificODCheckbox(specificODcheckBox);
		else
			specificODpanel.closeSpecificODCheckbox();
	}

	/**
	 * Opens the `OtherOptionsPanel`, allowing the user to configure additional
	 * simulation options.
	 */
	private void openParametersPanel() {
		ParametersPanel parametersFrame = new ParametersPanel();
		parametersFrame.setVisible(true); // Display the frame
	}

	/**
	 * Opens the `TestPanel, which enables the user to select testing options and
	 * parameters.
	 */
	private void openTestPanel() {
		TestPanel testPanel = new TestPanel();
		testPanel.setVisible(true); // Display the frame
	}

	/**
	 * Initiates the simulation with the selected parameters and starts the
	 * simulation process. This method sets the city name, simulation mode, and
	 * other parameters before running the simulation.
	 */
	private void startSimulation(boolean runInParallel) {

		Parameters.cityName = cityName.getSelectedItem();
		Parameters.stringMode = modeChoice.getSelectedItem();
		Parameters.defineMode();

		// Run the simulation with the updated parameters
		runSimulation(runInParallel);
	}

	/**
	 * Initiates the simulation with the selected parameters and starts the
	 * simulation process. This method sets the city name, simulation mode, and
	 * other parameters before running the simulation.
	 */
	private void runSimulation(boolean runInParallel) {

		try {
			Import importer = new Import();
			importer.importFiles();
		} catch (Exception e) {
			// Handle the exception or log an error message
			e.printStackTrace();
			// Additional handling logic...
		}

		Environment.prepare();
		LOGGER.info("Environment Prepared. About to Start Simulation");
		if (runInParallel) {
			remainingTripsCount = Parameters.empirical
					? Parameters.numAgents * Parameters.numberTripsPerAgent * Parameters.jobs * 3 // nr configurations
					: Parameters.numAgents * Parameters.numberTripsPerAgent * Parameters.jobs;
			updateRemainingTripsLabel(true);
			jobsLabel.setText("Parallelising " + Parameters.jobs + " Jobs");
			jobsLabel.setVisible(true);
			remainingTripsLabelParallel.setVisible(true);

		} else {
			remainingTripsCount = Parameters.empirical ? Parameters.numAgents * Parameters.numberTripsPerAgent * 3 // nr
																													// configurations
					: Parameters.numAgents * Parameters.numberTripsPerAgent;
			updateRemainingTripsLabel(false);
			jobLabel.setVisible(true);
			remainingTripsLabel.setVisible(true);
		}

		ArrayList<FlowHandler> flowHandlers = new ArrayList<>();

		if (runInParallel)
			IntStream.range(0, Parameters.jobs).parallel().forEach(job -> {
				final SimState state = new PedSimCity(System.currentTimeMillis(), job);
				state.start();
				List<Agent> agentList = ((PedSimCity) state).getAgentsList();
				while (state.schedule.step(state)) {
					remainingTripsCount = agentList.parallelStream()
							.mapToInt(agent -> agent.OD.size() - agent.tripsDone).sum() * Parameters.jobs;
					updateRemainingTripsLabel(true);
				}
				flowHandlers.add(((PedSimCity) state).flowHandler);
			});

		else {
			for (int job = 0; job < Parameters.jobs; job++) {
				jobLabel.setText("Executing Job Nr: " + job);
				final SimState state = new PedSimCity(System.currentTimeMillis(), job);
				state.start();
				List<Agent> agentList = ((PedSimCity) state).getAgentsList();
				while (state.schedule.step(state)) {
					remainingTripsCount = agentList.parallelStream()
							.mapToInt(agent -> agent.OD.size() - agent.tripsDone).sum();
					updateRemainingTripsLabel(false);
				}
				flowHandlers.add(((PedSimCity) state).flowHandler);
			}
		}

		handleEndSimulation();

	}

	private void handleEndSimulation() {
		Label endLabel = new Label("Simulation has ended. Close the window to exit.");
		endLabel.setBounds(10, 410, 300, 30);
		add(endLabel);
	}

	/**
	 * The main entry point for the PedSimCityApplet application.
	 *
	 * @param args an array of command-line arguments (not used in this
	 *             application).
	 */
	public static void main(String[] args) {
		PedSimCityApplet applet = new PedSimCityApplet();
		applet.addWindowListener(new WindowAdapter() {
			@Override
			public void windowClosing(WindowEvent e) {
				applet.dispose();
			}
		});
	}

	// This method updates the available options for cityName based on the selected
	// modeChoice
	private void updateCityNameOptions() {
		cityName.removeAll(); // Clear existing options

		// Get the selected modeChoice
		String selectedMode = modeChoice.getSelectedItem();

		// Add cityName options based on the selected mode
		if (selectedMode.equals("Testing Landmarks")) {
			cityName.add("London");
			cityName.add("Muenster");
		} else if (selectedMode.equals("Testing Urban Subdivisions")) {
			cityName.add("London");
			cityName.add("Paris");
			cityName.add("Muenster");
		} else if (selectedMode.equals("Empirical ABM")) {
			cityName.add("Muenster");
		} else if (selectedMode.equals("Testing Specific Route Choice Models")) {
			cityName.add("Muenster");
		}
		cityName.validate(); // Validate the layout to reflect changes in options
	}

	/**
	 * Invoked when an item's state changes. This method handles changes in the
	 * state of specificODcheckbox and modeChoice components.
	 *
	 * @param e an ItemEvent object that provides information about the event (e.g.,
	 *          which item's state changed).
	 */
	@Override
	public void itemStateChanged(ItemEvent e) {
		if (e.getSource() == specificODcheckBox)
			ODpanelStateChanged(e); // Call the ODpanelStateChanged method when specificODcheckbox state changes
		else if (e.getSource() == modeChoice)
			updateCityNameOptions();
		else
			Parameters.verboseMode = true;
	}

	/**
	 * Updates the text of the remainingTripsLabel to display the count of missing
	 * trips.
	 */
	private void updateRemainingTripsLabel(boolean runInParallel) {
		if (runInParallel)
			remainingTripsLabelParallel.setText("Trips left (jobs avg): " + remainingTripsCount);
		else
			remainingTripsLabel.setText("Trips left: " + remainingTripsCount);
	}
}
