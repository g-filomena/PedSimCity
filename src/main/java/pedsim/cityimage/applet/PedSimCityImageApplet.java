package pedsim.cityimage.applet;

import java.awt.Button;
import java.awt.Checkbox;
import java.awt.Choice;
import java.awt.Color;
import java.awt.Label;
import java.awt.event.ItemEvent;
import java.awt.event.ItemListener;
import java.awt.event.WindowAdapter;
import java.awt.event.WindowEvent;
import pedsim.cityimage.engine.CityImageEngine;
import pedsim.cityimage.engine.PedSimCityImage;
import pedsim.cityimage.parameters.TestPars;
import pedsim.core.applet.PedSimCityActionHandler;
import pedsim.core.applet.ServerProjectConfig;
import pedsim.core.engine.Engine;
import pedsim.core.engine.ScenarioConfig;
import pedsim.core.parameters.ParameterManager;
import pedsim.core.parameters.Pars;
import pedsim.core.utilities.LoggerUtil;

public class PedSimCityImageApplet extends pedsim.core.applet.PedSimCityApplet
    implements ItemListener {

  private static final long serialVersionUID = 1L;

  protected Choice simulationModeChoice;
  protected Button startButtonParallel;
  protected Checkbox specificODcheckBox;
  protected Checkbox verboseCheckBox;

  protected static Label jobLabel;
  protected static Label jobsLabel;
  protected static Label remainingTripsLabel;
  protected static Label remainingTripsLabelParallel;

  protected static int remainingTripsCount;

  public PedSimCityImageApplet() {
    super();

    addCityImageControls();
    refreshModeDependentControls();
    setSize(520, 620);
    validate();
    repaint();
  }

  private void addCityImageControls() {

    Label modeLabel = new Label("Simulation Mode:");
    modeLabel.setBounds(10, 40, 120, 20);
    add(modeLabel);

    simulationModeChoice = new Choice();
    simulationModeChoice.setBounds(140, 40, 230, 20);
    simulationModeChoice.add("Testing Landmarks");
    simulationModeChoice.add("Testing Urban Subdivisions");
    simulationModeChoice.add("Testing Specific Route Choice Models");
    simulationModeChoice.add("Empirical ABM");
    simulationModeChoice.addItemListener(this);
    add(simulationModeChoice);

    specificODcheckBox = new Checkbox("Testing Specific ODs");
    specificODcheckBox.setBounds(320, 180, 160, 25);
    specificODcheckBox.setEnabled(true);
    specificODcheckBox.addItemListener(this);
    add(specificODcheckBox);

    verboseCheckBox = new Checkbox("Verbose");
    verboseCheckBox.setBounds(320, 210, 100, 25);
    verboseCheckBox.setEnabled(true);
    verboseCheckBox.addItemListener(this);
    add(verboseCheckBox);

    startButtonParallel = new Button("Run in Parallel");
    startButtonParallel.setBounds(290, 330, 140, 50);
    startButtonParallel.setBackground(new Color(0, 220, 0));
    startButtonParallel.addActionListener(e -> {
      applyCityImageSelections(true);
      initialiseProgressLabels(true);
      launchSimulation(true);
    });
    add(startButtonParallel);

    jobLabel = new Label("Executing Job Nr:");
    jobLabel.setBounds(10, 390, 180, 20);
    jobLabel.setVisible(false);
    add(jobLabel);

    remainingTripsLabel = new Label("Trips left:");
    remainingTripsLabel.setBounds(10, 415, 180, 20);
    remainingTripsLabel.setVisible(false);
    add(remainingTripsLabel);

    jobsLabel = new Label("Parallelising jobs");
    jobsLabel.setBounds(220, 390, 180, 20);
    jobsLabel.setVisible(false);
    add(jobsLabel);

    remainingTripsLabelParallel = new Label("Trips left (jobs avg):");
    remainingTripsLabelParallel.setBounds(220, 415, 180, 20);
    remainingTripsLabelParallel.setVisible(false);
    add(remainingTripsLabelParallel);

    updateCityNameOptions();
  }

  @Override
  protected String getAppletTitle() {
    return "PedSimCity Image Applet";
  }

  @Override
  protected void updateCityNameOptions() {
    if (cityName == null) {
      return;
    }

    cityName.removeAll();

    String selectedMode =
        simulationModeChoice != null ? simulationModeChoice.getSelectedItem() : null;

    if ("Testing Landmarks".equals(selectedMode)) {
      cityName.add("London");
      cityName.add("Muenster");
    } else if ("Testing Urban Subdivisions".equals(selectedMode)) {
      cityName.add("London");
      cityName.add("Paris");
      cityName.add("Muenster");
    } else if ("Empirical ABM".equals(selectedMode)) {
      cityName.add("Muenster");
    } else if ("Testing Specific Route Choice Models".equals(selectedMode)) {
      cityName.add("Muenster");
    } else {
      cityName.add("Muenster");
    }

    cityName.validate();
  }

  @Override
  protected void configureStartButton(PedSimCityActionHandler handler) {
    startButton.addActionListener(e -> {
      applyCityImageSelections(false);
      initialiseProgressLabels(false);
      launchSimulation(false);
    });
  }

  @Override
  protected Engine.StateFactory buildStateFactory() {
    return PedSimCityImage::new;
  }

  @Override
  protected Engine buildEngine() {
    return new CityImageEngine(buildStateFactory());
  }

  protected void applyCityImageSelections(boolean runInParallel) {
    TestPars.cityName = getCityName();
    TestPars.stringMode = simulationModeChoice.getSelectedItem();
    TestPars.defineMode();
    TestPars.verboseMode = verboseCheckBox.getState();
    Pars.parallel = runInParallel;
  }

  protected void initialiseProgressLabels(boolean runInParallel) {
    if (runInParallel) {
      jobsLabel.setText("Parallelising " + getJobs() + " Jobs");
      jobsLabel.setVisible(true);
      remainingTripsLabelParallel.setVisible(true);
      updateRemainingTripsLabel(true);
    } else {
      jobLabel.setVisible(true);
      remainingTripsLabel.setVisible(true);
      updateRemainingTripsLabel(false);
    }
  }

  protected void refreshModeDependentControls() {
    if (simulationModeChoice == null) {
      return;
    }

    String selectedMode = simulationModeChoice.getSelectedItem();
    boolean routeChoiceMode = "Testing Specific Route Choice Models".equals(selectedMode);
    boolean empiricalMode = "Empirical ABM".equals(selectedMode);

    routeParsButton.setEnabled(routeChoiceMode);
    specificODcheckBox.setEnabled(!empiricalMode);

    if (empiricalMode) {
      specificODcheckBox.setState(false);
    }
  }

  protected void handleSpecificODCheckbox() {
    SpecificODpanel specificODpanel = new SpecificODpanel();
    if (specificODcheckBox.getState()) {
      specificODpanel.handleSpecificODCheckbox(specificODcheckBox);
    } else {
      specificODpanel.closeSpecificODCheckbox();
    }
  }

  @Override
  public void itemStateChanged(ItemEvent e) {
    Object source = e.getSource();

    if (source == simulationModeChoice) {
      updateCityNameOptions();
      refreshModeDependentControls();
    } else if (source == specificODcheckBox) {
      handleSpecificODCheckbox();
    } else if (source == verboseCheckBox) {
      TestPars.verboseMode = verboseCheckBox.getState();
    }
  }

  public static void setRemainingTripsCount(int count) {
    remainingTripsCount = count;
  }

  public static void updateRemainingTripsLabel(boolean runInParallel) {
    if (runInParallel) {
      if (remainingTripsLabelParallel != null) {
        remainingTripsLabelParallel.setText("Trips left: " + remainingTripsCount);
      }
    } else {
      if (remainingTripsLabel != null) {
        remainingTripsLabel.setText("Trips left: " + remainingTripsCount);
      }
    }
  }

  @Override
  protected ServerProjectConfig buildServerProjectConfig() {
    return new ServerProjectConfig("/mnt/home/gabriele/PedSimCity",
        "pedsim.cityimage.applet.PedSimCityImageApplet", "bin:lib/*:src/main/resources");
  }

  public static void main(String[] args) throws Exception {
    boolean headless = false;

    for (String arg : args) {
      if ("--headless".equals(arg) || arg.startsWith("--headless=")) {
        headless = true;
        break;
      }
    }

    if (headless) {
      LoggerUtil.getLogger().info("[SERVER] Running headless simulation...");
      ParameterManager.initFromArgsForServer(args);

      ScenarioConfig scenarioConfig = new ScenarioConfig(null, null);
      Engine engine = new CityImageEngine(PedSimCityImage::new);
      engine.runJobs(scenarioConfig, Pars.parallel);

    } else {
      PedSimCityImageApplet applet = new PedSimCityImageApplet();
      applet.addWindowListener(new WindowAdapter() {
        @Override
        public void windowClosing(WindowEvent e) {
          applet.dispose();
        }
      });
    }
  }
}
