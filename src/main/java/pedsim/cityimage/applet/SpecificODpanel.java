package pedsim.cityimage.applet;

import java.awt.Button;
import java.awt.Checkbox;
import java.awt.Frame;
import java.awt.Label;
import java.awt.TextField;
import java.awt.event.WindowAdapter;
import java.awt.event.WindowEvent;

import pedsim.cityimage.parameters.TestPars;

public class SpecificODpanel extends Frame {

	private static final long serialVersionUID = 1L;

	private static TextField originsField;
	private static TextField destinationsField;
	private static Frame specificODFrame;

	public void handleSpecificODCheckbox(Checkbox specificODcheckbox) {
		specificODFrame = new Frame("Testing Specific Routes");
		specificODFrame.setLayout(null);

		Label originsLabel = new Label("Origins:");
		originsLabel.setBounds(10, 40, 100, 20);
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
		saveButton.setBounds(10, 110, 80, 30);
		saveButton.addActionListener(event -> inputODs());
		specificODFrame.add(saveButton);

		Button cancelButton = new Button("Cancel");
		cancelButton.setBounds(110, 110, 80, 30);
		cancelButton.addActionListener(event -> {
			specificODcheckbox.setState(false);
			closeSpecificODCheckbox();
		});
		specificODFrame.add(cancelButton);

		specificODFrame.setSize(600, 170);
		specificODFrame.setVisible(true);

		specificODFrame.addWindowListener(new WindowAdapter() {
			@Override
			public void windowClosing(WindowEvent event) {
				specificODcheckbox.setState(false);
				closeSpecificODCheckbox();
			}
		});
	}

	public void closeSpecificODCheckbox() {
		if (specificODFrame != null) {
			specificODFrame.dispose();
			specificODFrame = null;
		}

		TestPars.testingSpecificOD = false;
		TestPars.originsTmp = new Integer[] {};
		TestPars.destinationsTmp = new Integer[] {};
	}

	private static void inputODs() {
		String originsText = originsField.getText().trim();
		String destinationsText = destinationsField.getText().trim();

		if (originsText.isEmpty() || destinationsText.isEmpty()) {
			showErrorMessage("Origins and destinations cannot be empty.");
			return;
		}

		String[] originsArray = originsText.split(",");
		String[] destinationsArray = destinationsText.split(",");

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
			showErrorMessage("Invalid input. Use comma-separated integer node IDs.");
			return;
		}

		TestPars.originsTmp = origins;
		TestPars.destinationsTmp = destinations;
		TestPars.testingSpecificOD = true;
		TestPars.numberTripsPerAgent = origins.length;

		if (specificODFrame != null) {
			specificODFrame.dispose();
			specificODFrame = null;
		}
	}

	private static void showErrorMessage(String message) {
		System.err.println("Specific OD error: " + message);
	}
}