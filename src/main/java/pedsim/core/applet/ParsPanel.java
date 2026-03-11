package pedsim.core.applet;

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
import java.util.stream.Collectors;
import pedsim.core.parameters.ParameterManager;
import pedsim.core.parameters.Pars;
import pedsim.core.parameters.RouteChoicePars;

/**
 * A graphical user interface panel for configuring general simulation parameters.
 */
public class ParsPanel extends Frame {
  private static final long serialVersionUID = 1L;
  private static final int X = 10;
  private int y = 50;
  private static final int Y_SPACE_BETWEEN = 30;

  // parameterName -> TextField
  public Map<String, TextField> doubleFields = new LinkedHashMap<>();
  public Map<String, TextField> booleanFields = new LinkedHashMap<>();
  public TextField cityCentreField = new TextField();

  String[][] doubleParamDefs =
      {{"metersPerDayPerPerson", "Average distance in meters walked a day, per person"},
          {"avgTripDistance", "Average Trip Distance between OD"}};

  Double[] defaults = {Pars.metersPerDayPerPerson, RouteChoicePars.avgTripDistance};

  String[][] booleanParamDefs = {{"parallel", "Run jobs in parallel"}};

  public ParsPanel() {
    super("Parameters Panel");
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

      boolean defaultValue = getBooleanDefault(key);

      TextField tf = addBooleanField(label, defaultValue, X, y);
      booleanFields.put(key, tf);
      y += Y_SPACE_BETWEEN;
    }

    // City centre regions special field
    Label cityCentreLabel = new Label("Enter City Centre Region IDs (comma-separated):");
    cityCentreLabel.setBounds(X, y, 350, 20);
    add(cityCentreLabel);

    String defaultCityCentreRegions = java.util.Arrays.stream(RouteChoicePars.cityCentreRegionsID)
        .map(String::valueOf).collect(Collectors.joining(","));
    cityCentreField = new TextField(defaultCityCentreRegions);
    cityCentreField.setBounds(X + 500, y, 200, 20);
    add(cityCentreField);

    y += Y_SPACE_BETWEEN;

    // Apply Button
    Button applyButton = new Button("Apply");
    applyButton.setBounds(X, y, 80, 30);
    applyButton.addActionListener(new ActionListener() {
      @Override
      public void actionPerformed(ActionEvent e) {
        // Update pars with values from fields
        ParameterManager.applyAll(Pars.class, doubleFields, booleanFields);

        // cityCentreRegions
        String text = cityCentreField.getText().trim();
        if (!text.isEmpty()) {
          ParameterManager.setFieldValue(RouteChoicePars.class, "cityCentreRegionsID", text);
        }

        closePanel();
      }
    });
    add(applyButton);

    setSize(800, 350);
    setVisible(true);

    addWindowListener(new WindowAdapter() {
      @Override
      public void windowClosing(WindowEvent e) {
        closePanel();
      }
    });
  }

  /**
   * Adds double-interpreter field to the panel for adjusting simulation parameters.
   *
   * @param fieldName The name of the parameter.
   * @param defaultValue The default value for the parameter.
   * @param x The x-coordinate for the field.
   * @param y The y-coordinate for the field.
   */
  private TextField addDoubleField(String fieldName, double defaultValue, int x, int y) {
    Label label = new Label(fieldName + ":");
    TextField textField = new TextField(Double.toString(defaultValue));
    label.setBounds(x, y, 600, 20);
    textField.setBounds(x + 600, y, 100, 20);
    add(label);
    add(textField);
    return textField;
  }

  private TextField addBooleanField(String fieldName, boolean defaultValue, int x, int y) {
    Label label = new Label(fieldName + ":");
    TextField textField = new TextField(Boolean.toString(defaultValue));
    label.setBounds(x, y, 600, 20);
    textField.setBounds(x + 600, y, 100, 20);
    add(label);
    add(textField);
    return textField;
  }

  private boolean getBooleanDefault(String fieldName) {
    try {
      return Pars.class.getDeclaredField(fieldName).getBoolean(null);
    } catch (Exception e) {
      return false;
    }
  }

  public static void main(String[] args) {
    ParsPanel frame = new ParsPanel();
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
