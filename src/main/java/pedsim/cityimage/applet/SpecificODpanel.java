package pedsim.cityimage.applet;

import java.awt.Button;
import java.awt.Checkbox;
import java.awt.Frame;
import java.awt.Label;
import java.awt.TextField;
import java.awt.event.WindowAdapter;
import java.awt.event.WindowEvent;

import pedsim.cityimage.parameters.TestPars;

/** Panel for manually entering specific origin-destination pairs. */
public class SpecificODpanel extends Frame {

	private static final long serialVersionUID = 1L;

	private static TextField originsField;
	private static TextField destinationsField;
	private static Frame specificODFrame;

	public void handleSpecificODCheckbox(Checkbox specificODcheckbox) {
		specificODFrame = new Frame("Testing Specific Routes");
		specificODFrame.setLayout(null);

		Label originsLabel = new Label("Origins:");
		originsLabel.setBounds(10, 40, 80, 20);
		specificODFrame.add(originsLabel);

		originsField = new TextField();
		originsField.setBounds(140, 40, 400, 20);
		specificODFrame.add(originsField);

		Label destinationsLabel = new Label("Destinations:");
		destinationsLabel.setBounds(10, 70, 100, 20);
		specificODFrame.add(destinationsLabel);

		destinationsField = new TextField();
		destinationsField.setBounds(140, 70, 400, 20);
		specificODFrame.add(destinationsField);

		Button saveButton = new Button("Save");
		saveButton.setBounds(10, 100, 80, 30);
		saveButton.addActionListener(event -> inputODs());
		specificODFrame.add(saveButton);

		specificODFrame.setSize(600, 150);
		specificODFrame.setVisible(true);

		specificODFrame.addWindowListener(new WindowAdapter() {
			@Override
			public void windowClosing(WindowEvent event) {
				specificODcheckbox.setState(false);
				specificODFrame.dispose();
			}
		});
	}

	public void closeSpecificODCheckbox() {
		if (specificODFrame != null) {
			specificODFrame.dispose();
			specificODFrame = null;
		}

		TestPars.testingSpecificOD = false;
	}

	private static void inputODs() {
		String[] originsArray = originsField.getText().split(",");
		String[] destinationsArray = destinationsField.getText().split(",");

		if (originsArray.length != destinationsArray.length) {
			showErrorMessage("The number of origins must match the number of destinations.");
			return;
		}

		Integer[] origins = new Integer[originsArray.length];
		Integer[] destinations = new Integer[destinationsArray.length];

		try {
			for (int i = 0; i < originsArray.length; i++) {
				origins[i] = Integer.parseInt(originsArray[i].trim());
				destinations[i] = Integer.parseInt(destinationsArray[i].trim());
			}

		} catch (NumberFormatException exception) {
			showErrorMessage("Invalid input. Please enter valid integer node IDs.");
			return;
		}

		TestPars.originsTmp = origins;
		TestPars.destinationsTmp = destinations;
		TestPars.testingSpecificOD = true;

		if (specificODFrame != null) {
			specificODFrame.dispose();
		}
	}

	private static void showErrorMessage(String message) {
		System.err.println("Specific OD error: " + message);
	}
}