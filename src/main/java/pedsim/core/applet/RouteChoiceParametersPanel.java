package pedsim.core.applet;

import java.awt.Button;
import java.awt.Font;
import java.awt.Frame;
import java.awt.Label;
import java.awt.TextField;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.awt.event.WindowAdapter;
import java.awt.event.WindowEvent;
import java.util.LinkedHashMap;
import java.util.Map;
import pedsim.core.parameters.ParameterManager;
import pedsim.core.parameters.RouteChoicePars;

/**
 * GUI panel for configuring RouteChoicePars.
 */
public class RouteChoiceParametersPanel extends Frame {
  private static final long serialVersionUID = 1L;
  private static final int X = 10;
  private int Y = 50;
  private static final int Y_SPACE_BETWEEN = 30;

  // parameterName -> TextField
  public Map<String, TextField> doubleFields = new LinkedHashMap<>();
  public Map<String, TextField> booleanFields = new LinkedHashMap<>();
  public TextField localPathField = new TextField(null);

  String[][] doubleParamDefs = {
      {"distanceNodeLandmark", "Max distance of a candidate Local Landmark from a Node (m)"},
      {"distanceAnchors", "Max distance of a Global Landmark (as anchor) from a Node"},
      {"nrAnchors", "Max Number of Anchors to be identified for each Destination"},
      {"threshold3dVisibility",
          "Min distance Between a node and the agent destination for considering 3D Visibility"},
      {"globalLandmarkThresholdCommunity",
          "Min score for a building to be considered a Global Landmark [0.0 - 0.95]"},
      {"localLandmarkThresholdCommunity",
          "Min score for a building to be considered a Local Landmark [0.0 - 0.95]"},
      {"salientNodesPercentile", "Percentile for choosing Salient Nodes [0.0 - 1.0]"},
      {"wayfindingEasinessThresholdCommunity", "Wayfinding Easiness Threshold [0.0 - 1.0]"},
      {"globalLandmarknessWeightDistanceCommunity",
          "Weight Global Landmarkness with Distance Cost [0.0 - 1.0]"},
      {"globalLandmarknessWeightAngularCommunity",
          "Weight Global Landmarkness with Angular Cost [0.0 - 1.0]"},
      {"regionNavActivationThreshold",
          "Minimum distance necessary to activate Region-based navigation (m)"},
      {"wayfindingEasinessThresholdRegionsCommunity", "Wayfinding Easiness Threshold in regions"}};

  Double[] defaults = {RouteChoicePars.distanceNodeLandmark, RouteChoicePars.distanceAnchors,
      (double) RouteChoicePars.nrAnchors, RouteChoicePars.threshold3dVisibility,
      RouteChoicePars.globalLandmarkThresholdCommunity,
      RouteChoicePars.localLandmarkThresholdCommunity, RouteChoicePars.salientNodesPercentile,
      RouteChoicePars.wayfindingEasinessThresholdCommunity,
      RouteChoicePars.globalLandmarknessWeightDistanceCommunity,
      RouteChoicePars.globalLandmarknessWeightAngularCommunity,
      RouteChoicePars.regionNavActivationThreshold,
      RouteChoicePars.wayfindingEasinessThresholdRegionsCommunity};

  public RouteChoiceParametersPanel() {
    super("RouteChoice Parameters Panel");
    setLayout(null);

    for (int i = 0; i < doubleParamDefs.length; i++) {
      String key = doubleParamDefs[i][0];
      String label = doubleParamDefs[i][1];
      double defaultValue = defaults[i];

      TextField tf = addDoubleField(label, defaultValue, X, Y);
      doubleFields.put(key, tf);
      Y += Y_SPACE_BETWEEN;
    }

    Label localPathLabel = new Label(
        "Fill only if running as Java Project, not from JAR (e.g.: C:/Users/you/.../resources/)");
    localPathLabel.setBounds(10, 480, 700, 20);
    localPathLabel.setFont(new Font("Arial", Font.ITALIC, 12));
    add(localPathLabel);

    localPathField.setBounds(360, 500, 350, 20);
    add(localPathField);

    Button applyButton = new Button("Apply");
    applyButton.setBounds(10, 550, 80, 30);
    applyButton.addActionListener(new ActionListener() {
      @Override
      public void actionPerformed(ActionEvent e) {
        ParameterManager.applyAll(RouteChoicePars.class, doubleFields, booleanFields);
        closePanel();
      }
    });
    add(applyButton);

    setSize(800, 600);
    setVisible(true);

    addWindowListener(new WindowAdapter() {
      @Override
      public void windowClosing(WindowEvent e) {
        closePanel();
      }
    });
  }

  private TextField addDoubleField(String fieldName, double defaultValue, int x, int y) {
    Label label = new Label(fieldName + ":");
    TextField textField = new TextField(Double.toString(defaultValue));
    label.setBounds(x, y, 600, 20);
    textField.setBounds(x + 600, y, 100, 20);
    add(label);
    add(textField);
    return textField;
  }

  public static void main(String[] args) {
    RouteChoiceParametersPanel frame = new RouteChoiceParametersPanel();
    frame.addWindowListener(new WindowAdapter() {
      @Override
      public void windowClosing(WindowEvent e) {
        System.exit(0);
      }
    });
  }

  private void closePanel() {
    setVisible(false);
    dispose();
  }
}
