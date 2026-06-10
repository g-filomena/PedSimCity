package pedsim.night.applet;

import java.awt.Button;
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
import pedsim.night.parameters.NightPars;

/**
 * A graphical user interface panel for configuring night-specific simulation parameters.
 */
public class NightParametersPanel extends Frame {
  private static final long serialVersionUID = 1L;
  private static final int X = 10;
  private int y = 50;
  private static final int Y_SPACE_BETWEEN = 30;

  public Map<String, TextField> doubleFields = new LinkedHashMap<>();
  public Map<String, TextField> booleanFields = new LinkedHashMap<>();

  String[][] doubleParamDefs = {
      {"minVulnerableLightSensitivity", "Min light sensitivity for vulnerable agents (Lux)"},
      {"maxVulnerableLightSensitivity", "Max light sensitivity for vulnerable agents (Lux)"},
      {"nonVulnerableLightSensitivity", "Light sensitivity for non-vulnerable agents (Lux)"},
      {"crowdednessPercentile", "Crowdedness Threshold Percentile (e.g. 80.0)"}
  };

  Double[] defaults = {
      NightPars.minVulnerableLightSensitivity,
      NightPars.maxVulnerableLightSensitivity,
      NightPars.nonVulnerableLightSensitivity,
      NightPars.crowdednessPercentile
  };

  String[][] booleanParamDefs = {
      {"enableLightABTesting", "Enable Light A/B Testing Logs"}
  };

  public NightParametersPanel() {
    super("Night Parameters Panel");
    setLayout(null);

    // Build doubles
    for (int i = 0; i < doubleParamDefs.length; i++) {
      String key = doubleParamDefs[i][0];
      String label = doubleParamDefs[i][1];
      double defaultValue = defaults[i];

      TextField tf = addDoubleField(label, defaultValue, X, y);
      doubleFields.put(key, tf);
      y += Y_SPACE_BETWEEN;
    }

    // Build booleans
    for (String[] def : booleanParamDefs) {
      String key = def[0];
      String label = def[1];
      boolean defaultValue = NightPars.enableLightABTesting;

      TextField tf = addBooleanField(label, defaultValue, X, y);
      booleanFields.put(key, tf);
      y += Y_SPACE_BETWEEN;
    }

    y += Y_SPACE_BETWEEN;

    // Apply Button
    Button applyButton = new Button("Apply");
    applyButton.setBounds(X, y, 80, 30);
    applyButton.addActionListener(new ActionListener() {
      @Override
      public void actionPerformed(ActionEvent e) {
        ParameterManager.applyAll(NightPars.class, doubleFields, booleanFields);
        closePanel();
      }
    });
    add(applyButton);

    setSize(750, 300);
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
    label.setBounds(x, y, 550, 20);
    textField.setBounds(x + 550, y, 100, 20);
    add(label);
    add(textField);
    return textField;
  }

  private TextField addBooleanField(String fieldName, boolean defaultValue, int x, int y) {
    Label label = new Label(fieldName + ":");
    TextField textField = new TextField(Boolean.toString(defaultValue));
    label.setBounds(x, y, 550, 20);
    textField.setBounds(x + 550, y, 100, 20);
    add(label);
    add(textField);
    return textField;
  }

  private void closePanel() {
    setVisible(false);
    dispose();
  }
}
