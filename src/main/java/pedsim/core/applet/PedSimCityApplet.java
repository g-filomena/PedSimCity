package pedsim.core.applet;

import java.awt.Button;
import java.awt.Choice;
import java.awt.Color;
import java.awt.Frame;
import java.awt.Label;
import java.awt.TextArea;
import java.awt.TextField;
import java.awt.event.WindowAdapter;
import java.awt.event.WindowEvent;
import pedsim.core.engine.Engine;
import pedsim.core.engine.Engine.StateFactory;
import pedsim.core.engine.PedSimCity;
import pedsim.core.engine.ScenarioConfig;
import pedsim.core.parameters.ParameterManager;
import pedsim.core.parameters.Pars;
import pedsim.core.utilities.LoggerUtil;
import pedsim.core.utilities.StringEnum;

public class PedSimCityApplet extends Frame {

  private static final long serialVersionUID = 1L;

  protected Choice cityName;
  Button startButton;
  Button runServerButton;
  Button configButton;
  Button endButton;
  protected Button routeParsButton;
  protected Button otherParsButton;
  private TextField daysTextField;
  private TextField jobsTextField;
  private TextField populationTextField;
  private TextField percentageTextField;
  private TextArea logArea;

  private boolean runningOnServer = false;
  private Thread simulationThread;

  // keep references if user opens them
  private RouteChoiceParametersPanel routePanel;
  private ParsPanel otherParsPanel;

  public PedSimCityApplet() {
    super();
    setTitle(getAppletTitle());
    setLayout(null);

    // --- GUI fields ---
    Label cityNameLabel = new Label("City Name:");
    cityNameLabel.setBounds(10, 70, 80, 20);
    add(cityNameLabel);

    cityName = new Choice();
    cityName.setBounds(140, 70, 150, 20);
    updateCityNameOptions();
    add(cityName);

    Label daysLabel = new Label("Duration in days:");
    daysTextField = new TextField("7");
    daysLabel.setBounds(10, 100, 120, 20);
    daysTextField.setBounds(190, 100, 100, 20);
    add(daysLabel);
    add(daysTextField);

    Label populationLabel = new Label("Actual Population:");
    populationTextField = new TextField("100000");
    populationLabel.setBounds(10, 130, 120, 20);
    populationTextField.setBounds(190, 130, 100, 20);
    add(populationLabel);
    add(populationTextField);

    Label percentageLabel = new Label("% Represented by Agents:");
    percentageTextField = new TextField("0.01");
    percentageLabel.setBounds(10, 160, 150, 20);
    percentageTextField.setBounds(190, 160, 100, 20);
    add(percentageLabel);
    add(percentageTextField);

    Label nrJobsLabel = new Label("Jobs:");
    jobsTextField = new TextField("1");
    nrJobsLabel.setBounds(10, 190, 100, 20);
    jobsTextField.setBounds(190, 190, 100, 20);
    add(nrJobsLabel);
    add(jobsTextField);

    // --- Buttons ---
    startButton = new Button("Run Simulation");
    startButton.setBounds(10, 330, 120, 50);
    startButton.setBackground(new Color(0, 220, 0));
    add(startButton);

    runServerButton = new Button("Run on Server");
    runServerButton.setBounds(150, 330, 120, 50);
    runServerButton.setBackground(new Color(0, 150, 200));
    add(runServerButton);

    endButton = new Button("End Simulation");
    endButton.setBackground(Color.PINK);

    configButton = new Button("Server Settings");
    configButton.setBounds(150, 280, 120, 40);
    configButton.setBackground(new Color(200, 200, 0));
    add(configButton);

    // Parameter panel buttons
    routeParsButton = new Button("Route Parameters");
    routeParsButton.setBounds(320, 100, 150, 30);
    routeParsButton.setBackground(new Color(180, 220, 250));
    routeParsButton.addActionListener(e -> openRoutePanel());
    add(routeParsButton);

    otherParsButton = new Button("Other Parameters");
    otherParsButton.setBounds(320, 140, 150, 30);
    otherParsButton.setBackground(new Color(220, 200, 250));
    otherParsButton.addActionListener(e -> openOtherParsPanel());
    add(otherParsButton);

    // --- Log area ---
    logArea = new TextArea("", 10, 80, TextArea.SCROLLBARS_VERTICAL_ONLY);
    logArea.setEditable(false);
    logArea.setBounds(10, 400, 460, 80);
    add(logArea);

    // --- Handlers ---
    ServerLauncherApplet serverLauncher = new ServerLauncherApplet(buildServerProjectConfig());
    PedSimCityActionHandler handler = new PedSimCityActionHandler(this, serverLauncher);

    startButton.addActionListener(handler.runLocalListener());
    runServerButton.addActionListener(handler.runServerListener());
    configButton.addActionListener(e -> serverLauncher.openConfigPanel());
    endButton.addActionListener(handler.endListener());

    // Redirect logger output to both console + TextArea
    LoggerUtil.redirectToTextArea(logArea);

    setSize(500, 520);
    setVisible(true);
  }

  protected void updateCityNameOptions() {
    cityName.removeAll();
    cityName.add("Muenster");
    cityName.validate();
  }

  // --- Logging utility ---
  public void appendLog(String msg) {
    LoggerUtil.getLogger().info(msg);
    if (logArea != null) {
      logArea.append(msg + "\n");
    }
  }

  // --- Open parameter panels ---
  private void openRoutePanel() {
    if (routePanel == null)
      routePanel = new RouteChoiceParametersPanel();
    routePanel.setVisible(true);
  }

  private void openOtherParsPanel() {
    if (otherParsPanel == null)
      otherParsPanel = new ParsPanel();
    otherParsPanel.setVisible(true);
  }

  // ---------------------------------------------------
  // Main entry point
  // ---------------------------------------------------
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
      StateFactory stateFactory = PedSimCity::new;

      Engine engine = new Engine(stateFactory);
      engine.runJobs(scenarioConfig, Pars.parallel);

    } else {
      PedSimCityApplet applet = new PedSimCityApplet();
      applet.addWindowListener(new WindowAdapter() {
        @Override
        public void windowClosing(WindowEvent e) {
          applet.dispose();
        }
      });
    }
  }

  protected ServerProjectConfig buildServerProjectConfig() {
    return new ServerProjectConfig("/mnt/home/gabriele/PedSimCity",
        "pedsim.core.applet.PedSimCityApplet", "bin:lib/*:src/main/resources");
  }

  // =====================================================
  // Getters & Setters
  // =====================================================
  protected String getAppletTitle() {
    return "PedSimCity Applet";
  }

  public String getCityName() {
    return cityName.getSelectedItem();
  }

  public String getDays() {
    return daysTextField.getText();
  }

  public String getPopulation() {
    return populationTextField.getText();
  }

  public String getPercentage() {
    return percentageTextField.getText();
  }

  public String getJobs() {
    return jobsTextField.getText();
  }

  public void setRunningOnServer(boolean value) {
    this.runningOnServer = value;
  }

  public boolean isRunningOnServer() {
    return runningOnServer;
  }

  public void setSimulationThread(Thread t) {
    this.simulationThread = t;
  }

  public Thread getSimulationThread() {
    return simulationThread;
  }

  public RouteChoiceParametersPanel getRoutePanel() {
    return routePanel;
  }

  public ParsPanel getOtherParsPanel() {
    return otherParsPanel;
  }

  protected ScenarioConfig buildScenarioConfig() {
    return new ScenarioConfig(StringEnum.Learner.values(), null);
  }

  protected Engine.StateFactory buildStateFactory() {
    return PedSimCity::new;
  }

}
