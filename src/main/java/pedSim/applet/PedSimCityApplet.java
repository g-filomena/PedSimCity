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
import java.util.logging.Logger;

import pedSim.engine.Environment;
import pedSim.engine.Import;
import pedSim.engine.Parameters;
import pedSim.engine.PedSimCity;
import sim.engine.SimState;
import sim.graph.EdgeGraph;

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
	private Checkbox specificODcheckbox;
	private Label jobsDoneLabel;
	private Label remainingTripsLabel;
	private int remainingTripsCount = 0;
	private static final Logger LOGGER = Logger.getLogger(Import.class.getName());

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

		specificODcheckbox = new Checkbox("Testing Specific ODs");
		specificODcheckbox.setBounds(10, 100, 300, 40);
		specificODcheckbox.addItemListener(this);
		add(specificODcheckbox);

		Button otherOptionsButton = new Button("Other Options");
		otherOptionsButton.setBounds(10, 150, 150, 30);
		otherOptionsButton.addActionListener(new ActionListener() {
			public void actionPerformed(ActionEvent e) {
				openParametersPanel();
			}
		});
		add(otherOptionsButton);

		Button testingButton = new Button("Choose Route-Choices");
		testingButton.setEnabled(false);
		testingButton.setBounds(10, 200, 150, 30);
		testingButton.addActionListener(new ActionListener() {
			public void actionPerformed(ActionEvent e) {
				openTestPanel();
			}
		});
		modeChoice.addItemListener(new ItemListener() {
			public void itemStateChanged(ItemEvent e) {
				String selectedMode = modeChoice.getSelectedItem();
				// Enable the button only if the selected mode is "Testing Specific Route Choice
				// Models"
				testingButton.setEnabled(selectedMode.equals("Testing Specific Route Choice Models"));
			}
		});
		add(testingButton);

		startButton = new Button("Start Simulation");
		startButton.setBounds(10, 250, 120, 50);
		startButton.addActionListener(new ActionListener() {
			public void actionPerformed(ActionEvent e) {
				startSimulation();
			}
		});
		startButton.setBackground(Color.PINK);
		add(startButton);

		jobsDoneLabel = new Label("Job Number: 0");
		jobsDoneLabel.setBounds(140, 250, 120, 20);
		add(jobsDoneLabel);

		remainingTripsLabel = new Label("Missing Trips: 0");
		remainingTripsLabel.setBounds(140, 280, 120, 20);
		add(remainingTripsLabel);

		setSize(400, 350);
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
		if (specificODcheckbox.getState())
			specificODpanel.handleSpecificODCheckbox(specificODcheckbox);
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
	private void startSimulation() {

		Parameters.cityName = cityName.getSelectedItem();
		Parameters.stringMode = modeChoice.getSelectedItem();
		Parameters.defineMode();
		updateMissingTripsLabel();

		// Run the simulation with the updated parameters
		runSimulation();
	}

	/**
	 * Initiates the simulation with the selected parameters and starts the
	 * simulation process. This method sets the city name, simulation mode, and
	 * other parameters before running the simulation.
	 */
	private void runSimulation() {

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
		int jobTrips = Parameters.numAgents * Parameters.numberTripsPerAgent;
		if (Parameters.empirical)
			jobTrips *= 3;

		for (int job = 0; job < Parameters.jobs; job++) {
			jobsDoneLabel.setText("Job Number: " + job);
			for (final EdgeGraph edge : PedSimCity.network.getEdges())
				edge.resetVolumes();
			final SimState state = new PedSimCity(System.currentTimeMillis(), job);
			remainingTripsCount = jobTrips;
			updateMissingTripsLabel();
			state.start();
			while (state.schedule.step(state)) {
				remainingTripsCount = PedSimCity.agentsList.parallelStream()
						.mapToInt(agent -> agent.OD.size() - agent.tripsDone).sum();
				updateMissingTripsLabel();
			}
		}
		System.exit(0);
	}

	/**
	 * The main entry point for the PedSimCityApplet application.
	 *
	 * @param args an array of command-line arguments (not used in this
	 *             application).
	 */
	public static void main(String[] args) {
		PedSimCityApplet applet = new PedSimCityApplet();
		applet.addWindowListener(new java.awt.event.WindowAdapter() {
			public void windowClosing(java.awt.event.WindowEvent e) {
				System.exit(0);
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
		if (e.getSource() == specificODcheckbox) {
			ODpanelStateChanged(e); // Call the ODpanelStateChanged method when specificODcheckbox state changes
		} else if (e.getSource() == modeChoice) {
			updateCityNameOptions();
		}
	}

	/**
	 * Updates the text of the missingTripsLabel to display the count of missing
	 * trips.
	 */
	private void updateMissingTripsLabel() {
		remainingTripsLabel.setText("Missing Trips: " + remainingTripsCount);
	}
}
