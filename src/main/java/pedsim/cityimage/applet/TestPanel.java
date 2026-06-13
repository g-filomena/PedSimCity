package pedsim.cityimage.applet;

import java.awt.Button;
import java.awt.Checkbox;
import java.awt.Frame;
import java.awt.Label;
import java.awt.TextField;
import java.util.ArrayList;
import java.util.List;

import pedsim.cityimage.parameters.TestPars;
import pedsim.cityimage.utilities.StringEnum.RouteChoice;
import pedsim.core.parameters.Pars;

/** GUI panel for selecting route-choice models to test. */
public class TestPanel extends Frame {

	private static final long serialVersionUID = 1L;

	private final List<RouteChoice> selectedChoices = new ArrayList<>();

	private TextField jobsTextField;
	private TextField numTripsPerAgentField;

	public TestPanel() {
		setTitle("Testing panel");
		setLayout(null);

		Button saveButton = new Button("Save");
		saveButton.addActionListener(event -> getTestParameters());

		int y = 30;

		for (RouteChoice choice : RouteChoice.values()) {
			Checkbox checkbox = new Checkbox(choice.toString());
			checkbox.setName(choice.name());
			checkbox.setBounds(10, y, 400, 20);
			y += 30;
			add(checkbox);
		}

		Label jobsLabel = new Label("Jobs:");
		jobsLabel.setBounds(10, y, 80, 20);
		add(jobsLabel);

		jobsTextField = new TextField(Integer.toString(Pars.jobs));
		jobsTextField.setBounds(170, y, 100, 20);
		add(jobsTextField);

		y += 30;

		Label numTripsPerAgentLabel = new Label("Number of Trips per Agent:");
		numTripsPerAgentLabel.setBounds(10, y, 155, 20);
		add(numTripsPerAgentLabel);

		numTripsPerAgentField = new TextField(Integer.toString(TestPars.numberTripsPerAgent));
		numTripsPerAgentField.setBounds(170, y, 100, 20);
		add(numTripsPerAgentField);

		y += 30;

		saveButton.setBounds(10, y, 80, 30);
		add(saveButton);

		setSize(430, 720);
		setVisible(true);
	}

	public void getTestParameters() {
		selectedChoices.clear();

		for (int i = 0; i < getComponentCount(); i++) {
			if (getComponent(i) instanceof Checkbox checkbox && checkbox.getState()) {
				selectedChoices.add(RouteChoice.valueOf(checkbox.getName()));
			}
		}

		if (selectedChoices.isEmpty()) {
			selectedChoices.add(RouteChoice.ROAD_DISTANCE);
			selectedChoices.add(RouteChoice.ANGULAR_CHANGE);
		}

		TestPars.routeChoiceUser = selectedChoices.toArray(new RouteChoice[0]);

		try {
			Pars.jobs = Integer.parseInt(jobsTextField.getText().trim());
			TestPars.numberTripsPerAgent = Integer.parseInt(numTripsPerAgentField.getText().trim());

		} catch (NumberFormatException exception) {
			System.err.println("Invalid input for jobs or number of trips per agent.");
		}

		TestPars.stringMode = "Testing Specific Route Choice Models";
		TestPars.testingModels = true;
		TestPars.defineMode();

		dispose();
	}

	public List<RouteChoice> getSelectedChoices() {
		return selectedChoices;
	}
}