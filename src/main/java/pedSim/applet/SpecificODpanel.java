package pedSim.applet;

import java.awt.Button;
import java.awt.Checkbox;
import java.awt.Frame;
import java.awt.Label;
import java.awt.TextField;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.awt.event.WindowAdapter;
import java.awt.event.WindowEvent;

import pedSim.engine.Parameters;

/**
 * A GUI panel for handling specific origin-destination routes.
 */
public class SpecificODpanel extends Frame {

	private static final long serialVersionUID = 1L;
	private static TextField originsField;
	private static TextField destinationsField;
	private static Frame specificODFrame;

	/**
	 * Handles the specific origin-destination checkbox action.
	 *
	 * @param specificODcheckbox The checkbox that triggers this action.
	 */
	public void handleSpecificODCheckbox(Checkbox specificODcheckbox) {

		// Open a new window for entering origins and destinations
		specificODFrame = new Frame("Testing Specific Routes");
		specificODFrame.setLayout(null);

		Label originsLabel = new Label("Origins:");
		originsLabel.setBounds(10, 40, 80, 20);
		specificODFrame.add(originsLabel);
		originsField = new TextField();
		originsField.setBounds(140, 40, 400, 20);
		specificODFrame.add(originsField);

		Label destinationsLabel = new Label("Destinations:");
		destinationsLabel.setBounds(10, 70, 80, 20);
		specificODFrame.add(destinationsLabel);
		destinationsField = new TextField();
		destinationsField.setBounds(140, 70, 400, 20);
		specificODFrame.add(destinationsField);

		Button saveButton = new Button("Save");
		saveButton.setBounds(10, 100, 80, 30);
		saveButton.addActionListener(new ActionListener() {
			public void actionPerformed(ActionEvent e) {
				inputODs();
			}
		});
		specificODFrame.add(saveButton);
		specificODFrame.setSize(600, 150);
		specificODFrame.setVisible(true);

		specificODFrame.addWindowListener(new WindowAdapter() {
			@Override
			public void windowClosing(WindowEvent e) {
				// Update the state of the checkbox when the window is closed
				specificODcheckbox.setState(false);
				specificODFrame.dispose();
			}
		});
	}

	/**
	 * Closes the specific origin-destination input window.
	 */
	public void closeSpecificODCheckbox() {
		if (specificODFrame != null) {
			specificODFrame.dispose();
			specificODFrame = null;
		}
	}

	/**
	 * Parses user-input origins and destinations, validates them, and updates
	 * relevant parameters.
	 */
	private static void inputODs() {
		// Parse the values and update UserParameters
		String[] originsArray = originsField.getText().split(",");
		String[] destinationsArray = destinationsField.getText().split(",");

		// Validate the lengths of origins and destinations
		if (originsArray.length != destinationsArray.length) {
			showErrorMessage("The number of origins must match the number of destinations.");
			return;
		}

		Integer[] origins;
		Integer[] destinations;

		try {
			origins = new Integer[originsArray.length];
			destinations = new Integer[destinationsArray.length];

			for (int i = 0; i < originsArray.length; i++) {
				origins[i] = Integer.parseInt(originsArray[i].trim());
			}

			for (int i = 0; i < destinationsArray.length; i++) {
				destinations[i] = Integer.parseInt(destinationsArray[i].trim());
			}
		} catch (NumberFormatException ex) {
			showErrorMessage("Invalid input. Please enter valid integers for origins and destinations.");
			return;
		}

		Parameters.originsTmp = origins;
		Parameters.destinationsTmp = destinations;
		Parameters.testingSpecificOD = true;
		specificODFrame.dispose(); // Close the window
	}

	/**
	 * Displays an error message or alert with the given message.
	 *
	 * @param message The error message to display.
	 */
	private static void showErrorMessage(String message) {
		// Display an error message or show an alert
		System.err.println("Error: " + message);
	}
}
