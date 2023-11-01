package pedSim.applet;

import java.awt.Button;
import java.awt.Frame;
import java.awt.Label;
import java.awt.TextField;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.awt.event.WindowAdapter;
import java.awt.event.WindowEvent;
import java.awt.event.WindowListener;

import pedSim.engine.Parameters;

/**
 * A graphical user interface panel for configuring various simulation
 * parameters.
 */
public class ParametersPanel extends Frame {
	private static final long serialVersionUID = 1L;

	/**
	 * Constructs the Parameters Panel.
	 */
	public ParametersPanel() {
		super("Parameters Panel");
		setLayout(null);

		addField("Max distance of a candidate Local Landmark from a Node (m)", Parameters.distanceNodeLandmark, 10, 50);
		addField("Max distance of a Landmark from a Node", Parameters.distanceAnchors, 10, 80);
		addField("Max Number of Anchors to be identified for each Destination", Parameters.nrAnchors, 10, 110);
		addField("Min distance Between a node and the agent destination for considering 3D Visibility",
				Parameters.threshold3dVisibility, 10, 140);
		addField("Min score for a building to be considered a Global Landmark (0.0 - 1.0)",
				Parameters.globalLandmarkThreshold, 10, 170);
		addField("Min score for a building to be considered a Local Landmark (0.0 - 1.0)",
				Parameters.localLandmarkThreshold, 10, 200);
		addField("Percentile for choosing Salient Nodes (on centrality values) (0.0 - 1.0)",
				Parameters.salientNodesPercentile, 10, 230);
		addField("Wayfinding Easiness Threshold above which local landmarks are not identified",
				Parameters.wayfindingEasinessThreshold, 10, 260);
		addField("Weight Global Landmarkness in combination with Distance Edge Cost (0.0 - 1.0)",
				Parameters.globalLandmarknessWeightDistance, 10, 290);
		addField("Weight Global Landmarkness in combination with Angular Edge Cost (0.0 - 1.0)",
				Parameters.globalLandmarknessWeightAngular, 10, 320);
		addField("Minimum distance necessary to activate Region-based navigation (m)",
				Parameters.regionBasedNavigationThreshold, 10, 350);
		addField("Wayfinding Easiness Threshold above which local landmarks are not identified within regions",
				Parameters.wayfindingEasinessThresholdRegions, 10, 380);
		addBooleanField("Employ Subgraphs in Dijkstra Algorithm", Parameters.subGraph, 10, 410);
		addBooleanField("Identify Destinations based on DMA Approach", Parameters.usingDMA, 10, 440);

		Button applyButton = new Button("Apply");
		applyButton.setBounds(10, 490, 80, 30);
		applyButton.addActionListener(new ActionListener() {
			@Override
			public void actionPerformed(ActionEvent e) {
				adjustParameters();
				closePanel();
			}
		});
		add(applyButton);

		setSize(800, 600);
		setVisible(true);

		addWindowListener((WindowListener) new WindowAdapter() {
			@Override
			public void windowClosing(WindowEvent e) {
				closePanel();
			}
		});
	}

	/**
	 * Adds a labeled boolean field to the panel for adjusting simulation
	 * parameters.
	 *
	 * @param fieldName    The name of the parameter.
	 * @param defaultValue The default value for the parameter.
	 * @param x            The x-coordinate for the field.
	 * @param y            The y-coordinate for the field.
	 */
	private void addField(String fieldName, double defaultValue, int x, int y) {
		Label label = new Label(fieldName + ":");
		TextField textField = new TextField(Double.toString(defaultValue));
		label.setBounds(x, y, 600, 20);
		textField.setBounds(x + 600, y, 100, 20);
		add(label);
		add(textField);
	}

	/**
	 * Adds a labeled boolean field to the panel for adjusting simulation
	 * parameters.
	 *
	 * @param fieldName    The name of the parameter.
	 * @param defaultValue The default value for the parameter.
	 * @param x            The x-coordinate for the field.
	 * @param y            The y-coordinate for the field.
	 */
	private void addBooleanField(String fieldName, boolean defaultValue, int x, int y) {
		Label label = new Label(fieldName + ":");
		TextField textField = new TextField(Boolean.toString(defaultValue));
		label.setBounds(x, y, 600, 20);
		textField.setBounds(x + 600, y, 100, 20);
		add(label);
		add(textField);
	}

	public static void main(String[] args) {
		ParametersPanel frame = new ParametersPanel();
		frame.addWindowListener(new WindowAdapter() {
			@Override
			public void windowClosing(WindowEvent e) {
				System.exit(0);
			}
		});
	}

	/**
	 * Adjusts the simulation parameters based on the values entered in text fields.
	 * Parses the values and updates the corresponding parameters in the Parameters
	 * class.
	 */
	private void adjustParameters() {
		Parameters.distanceNodeLandmark = Double.parseDouble(getTextFieldValue(50));
		Parameters.distanceAnchors = Double.parseDouble(getTextFieldValue(80));
		Parameters.nrAnchors = (int) Double.parseDouble(getTextFieldValue(110));
		Parameters.threshold3dVisibility = Double.parseDouble(getTextFieldValue(140));
		Parameters.globalLandmarkThreshold = Double.parseDouble(getTextFieldValue(170));
		Parameters.localLandmarkThreshold = Double.parseDouble(getTextFieldValue(200));
		Parameters.salientNodesPercentile = Double.parseDouble(getTextFieldValue(230));
		Parameters.wayfindingEasinessThreshold = Double.parseDouble(getTextFieldValue(260));
		Parameters.globalLandmarknessWeightDistance = Double.parseDouble(getTextFieldValue(290));
		Parameters.globalLandmarknessWeightAngular = Double.parseDouble(getTextFieldValue(320));
		Parameters.regionBasedNavigationThreshold = Double.parseDouble(getTextFieldValue(350));
		Parameters.wayfindingEasinessThresholdRegions = Double.parseDouble(getTextFieldValue(380));
		Parameters.subGraph = Boolean.parseBoolean(getTextFieldValue(410));
		Parameters.usingDMA = Boolean.parseBoolean(getTextFieldValue(440));
	}

	/**
	 * Closes the Panel.
	 */
	private void closePanel() {
		setVisible(false);
		dispose();
	}

	/**
	 * Retrieves the value of a text field based on its y-coordinate.
	 *
	 * @param y The y-coordinate of the text field.
	 * @return The text entered in the text field or an empty string if not found.
	 */
	private String getTextFieldValue(int y) {
		TextField textField = null;
		for (int i = 0; i < getComponentCount(); i++) {
			if (getComponent(i) instanceof TextField && getComponent(i).getY() == y) {
				textField = (TextField) getComponent(i);
				break;
			}
		}
		return textField != null ? textField.getText() : "";
	}
}
