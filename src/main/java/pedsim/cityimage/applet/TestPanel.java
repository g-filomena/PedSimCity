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

public class TestPanel extends Frame {

  private static final long serialVersionUID = 1L;

  private final List<RouteChoice> selectedChoices = new ArrayList<>();

  private TextField jobsTextField;
  private TextField numTripsPerAgentField;

  public TestPanel() {
    setTitle("Testing Route Choice Models");
    setLayout(null);

    int y = 40;

    for (RouteChoice choice : RouteChoice.values()) {
      Checkbox checkbox = new Checkbox(choice.toString());
      checkbox.setName(choice.name());
      checkbox.setBounds(10, y, 380, 20);

      if (choice == RouteChoice.ROAD_DISTANCE || choice == RouteChoice.ANGULAR_CHANGE) {
        checkbox.setState(true);
      }

      add(checkbox);
      y += 28;
    }

    Label jobsLabel = new Label("Jobs:");
    jobsLabel.setBounds(10, y + 10, 150, 20);
    add(jobsLabel);

    jobsTextField = new TextField(Integer.toString(Pars.jobs));
    jobsTextField.setBounds(170, y + 10, 100, 20);
    add(jobsTextField);

    Label tripsLabel = new Label("Trips per agent:");
    tripsLabel.setBounds(10, y + 40, 150, 20);
    add(tripsLabel);

    numTripsPerAgentField = new TextField(Integer.toString(TestPars.numberTripsPerAgent));
    numTripsPerAgentField.setBounds(170, y + 40, 100, 20);
    add(numTripsPerAgentField);

    Button saveButton = new Button("Save");
    saveButton.setBounds(10, y + 80, 90, 30);
    saveButton.addActionListener(event -> saveTestParameters());
    add(saveButton);

    Button closeButton = new Button("Close");
    closeButton.setBounds(120, y + 80, 90, 30);
    closeButton.addActionListener(event -> dispose());
    add(closeButton);

    setSize(430, y + 160);
    setVisible(true);
  }

  private void saveTestParameters() {
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
    TestPars.stringMode = "Testing Specific Route Choice Models";

    try {
      Pars.jobs = Integer.parseInt(jobsTextField.getText().trim());
    } catch (NumberFormatException exception) {
      Pars.jobs = 1;
    }

    try {
      TestPars.numberTripsPerAgent = Integer.parseInt(numTripsPerAgentField.getText().trim());
    } catch (NumberFormatException exception) {
      TestPars.numberTripsPerAgent = 2;
    }

    TestPars.defineMode();

    dispose();
  }

  public List<RouteChoice> getSelectedChoices() {
    return selectedChoices;
  }
}
