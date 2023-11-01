package pedSim.applet;

import java.awt.Button;
import java.awt.Checkbox;
import java.awt.Frame;
import java.awt.Label;
import java.awt.TextField;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.util.ArrayList;
import java.util.List;

import pedSim.engine.Parameters;
import pedSim.utilities.StringEnum.RouteChoice;

/**
 * A graphical user interface (GUI) panel for setting and saving test
 * parameters.
 */
public class TestPanel extends Frame {

	private static final long serialVersionUID = 1L;
	private List<RouteChoice> selectedChoices;
	private TextField jobsTextField;
	private TextField numTripsPerAgentField;

	/**
	 * Initialises the TestPanel GUI with checkboxes and input fields.
	 */
	public TestPanel() {
		setTitle("Testing panel");
		setLayout(null);

		selectedChoices = new ArrayList<>();

		// Add an action listener to the Save button to get the selected choices
		Button saveButton = new Button("Save");
		saveButton.addActionListener(new ActionListener() {
			public void actionPerformed(ActionEvent e) {
				getTestParameters(e); // Call the method `getTestParameters()` when the button is clicked
			}
		});

		int value = 30;
		// Create checkboxes for each RouteChoice
		for (RouteChoice choice : RouteChoice.values()) {
			Checkbox checkbox = new Checkbox(choice.toString());
			checkbox.setName(choice.name());
			checkbox.setBounds(10, value, 400, 20);
			value += 30;
			add(checkbox);
		}

		// Additional components for setting user-defined parameters
		Label jobsLabel = new Label("Jobs:");
		add(jobsLabel);
		jobsLabel.setBounds(10, value, 80, 20);
		jobsTextField = new TextField();
		jobsTextField.setBounds(170, value, 100, 20);
		add(jobsTextField);
		value += 30;

		Label numTripsPerAgentLabel = new Label("Number of Trips per Agent:");
		add(numTripsPerAgentLabel);
		numTripsPerAgentLabel.setBounds(10, value, 155, 20);
		numTripsPerAgentField = new TextField();
		numTripsPerAgentField.setBounds(170, value, 100, 20);
		add(numTripsPerAgentField);
		value += 30;

		saveButton.setBounds(10, value, 80, 30);
		add(saveButton);
		setSize(400, 700);
		setVisible(true);
	}

	/**
	 * Retrieves the selected choices from checkboxes and user-defined parameters
	 * from input fields.
	 *
	 * @param e The ActionEvent triggered by the Save button.
	 */
	public void getTestParameters(ActionEvent e) {
		// Update the selectedChoices list based on the checkboxes
		selectedChoices.clear();

		for (int i = 0; i < getComponentCount(); i++) {
			if (getComponent(i) instanceof Checkbox) {
				Checkbox checkbox = (Checkbox) getComponent(i);
				if (checkbox.getState()) {
					selectedChoices.add(RouteChoice.valueOf(checkbox.getName()));
				}
			}
		}
		// Print the selected choices
		System.out.println("Selected Choices: " + selectedChoices);

		// Update user-defined parameters
		try {
			Parameters.routeChoiceUser = new RouteChoice[selectedChoices.size()];
			for (int counter = 0; counter < selectedChoices.size(); counter++) {
				Parameters.routeChoiceUser[counter] = selectedChoices.get(counter);
			}
			Parameters.jobs = Integer.parseInt(jobsTextField.getText());
			Parameters.numberTripsPerAgent = Integer.parseInt(numTripsPerAgentField.getText());
		} catch (NumberFormatException ex) {
			System.err.println("Invalid input for jobs or numAgents.");
		}
		Parameters.testingModels = true;
		this.dispose(); // Close the window
	}

	/**
	 * Gets the list of selected RouteChoice values.
	 *
	 * @return A list of selected RouteChoice values.
	 */
	public List<RouteChoice> getSelectedChoices() {
		return selectedChoices;
	}
}
