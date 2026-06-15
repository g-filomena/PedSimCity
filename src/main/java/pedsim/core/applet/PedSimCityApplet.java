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
import pedsim.core.engine.PedSimCity;
import pedsim.core.engine.ScenarioConfig;
import pedsim.core.engine.SimulationLauncher;
import pedsim.core.parameters.ParameterManager;
import pedsim.core.parameters.Pars;
import pedsim.core.utilities.LoggerUtil;
import pedsim.core.utilities.StringEnum;
import pedsim.core.website.SimulationRestApi;

public class PedSimCityApplet extends Frame {

  private static final long serialVersionUID = 1L;

  protected Choice cityName;
  protected Button startButton;
  protected Button runServerButton;
  protected Button configButton;
  protected Button endButton;
  protected Button routeParsButton;
  protected Button otherParsButton;

  private TextField daysTextField;
  private TextField jobsTextField;
  private TextField populationTextField;
  private TextField percentageTextField;
  private TextArea logArea;

  private boolean runningOnServer = false;
  private Thread simulationThread;

  private RouteChoiceParametersPanel routePanel;
  private ParsPanel otherParsPanel;

  public PedSimCityApplet() {
    super();
    setTitle(getAppletTitle());
    setLayout(null);

    // --- GUI fields --
    Label cityNameLabel = new Label("City Name:");
    cityNameLabel.setBounds(10, 70, 80, 20);
    add(cityNameLabel);

    cityName = new Choice();
    cityName.setBounds(140, 70, 150, 20);
    updateCityNameOptions();
    add(cityName);

    Label daysLabel = new Label("Duration in days:");
    daysTextField = new TextField("1");
    daysLabel.setBounds(10, 100, 120, 20);
    daysTextField.setBounds(190, 100, 100, 20);
    add(daysLabel);
    add(daysTextField);

    Label populationLabel = new Label("Actual Population:");
    populationTextField = new TextField("1000");
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

    // Parameter panel buttons
    configButton = new Button("Server Settings");
    configButton.setBounds(150, 280, 120, 40);
    configButton.setBackground(new Color(200, 200, 0));
    add(configButton);

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
    configureStartButton(handler);

    // --- Action Listeners ---
    runServerButton.addActionListener(handler.runServerListener());
    configButton.addActionListener(e -> serverLauncher.openConfigPanel());
    endButton.addActionListener(handler.endListener());

    LoggerUtil.redirectToTextArea(logArea);

    setSize(500, 520);
    setVisible(true);
  }

  protected void updateCityNameOptions() {
    cityName.removeAll();
    cityName.add("Muenster");
    cityName.add("TorinoCentre");
    cityName.validate();
  }

  /**
   * @param handler
   */
  protected void configureStartButton(PedSimCityActionHandler handler) {
    startButton.addActionListener(e -> launchSimulation(false));
  }

  protected void launchSimulation(boolean runInParallel) {
    setRunningOnServer(false);

    // Read the GUI text fields into Pars BEFORE building the engine
    ParameterManager.collectParameters(this);
    Pars.setSimulationParameters();

    final ScenarioConfig scenarioConfig = buildScenarioConfig();
    final Engine engine = buildEngine();

    Thread thread = new Thread(() -> {
      try {
        engine.runJobs(scenarioConfig, runInParallel);
      } catch (Exception ex) {
        LoggerUtil.getLogger().severe("Simulation failed: " + ex.getMessage());
        ex.printStackTrace();
      }
    }, runInParallel ? "pedsim-parallel-simulation" : "pedsim-local-simulation");

    setSimulationThread(thread);
    thread.start();
  }

  public void appendLog(String msg) {
    LoggerUtil.getLogger().info(msg);
    if (logArea != null) {
      logArea.append(msg + "\n");
    }
  }

  private void openRoutePanel() {
    if (routePanel == null) {
      routePanel = new RouteChoiceParametersPanel();
    }
    routePanel.setVisible(true);
  }

  private void openOtherParsPanel() {
    if (otherParsPanel == null) {
      otherParsPanel = new ParsPanel();
    }
    otherParsPanel.setVisible(true);
  }

  // ---------------------------------------------------
  // Main entry point
  // ---------------------------------------------------
  public static void main(String[] args) throws Exception {
    boolean headless = false;
    boolean forceWebsite = false;
    boolean forceGui = false;

    for (String arg : args) {
      if ("--headless".equals(arg) || arg.startsWith("--headless=")) headless = true;
      if ("--website".equals(arg)) forceWebsite = true;
      if ("--gui".equals(arg)) forceGui = true;
    }

    if (headless) {
      LoggerUtil.getLogger().info("[SERVER] Running headless simulation...");
      coreLauncher().headlessRun(args);
      return;
    }

    // Interactive selection if no flags provided
    int choice = 0;
    if (forceWebsite) choice = 2;
    else if (forceGui) choice = 1;
    else {
      System.out.println("\n========================================");
      System.out.println("   PedSimCity Core - Startup Options    ");
      System.out.println("========================================");
      System.out.println(" 1. Run Standard GUI (AWT)");
      System.out.println(" 2. Start REST API for External Dashboard (Streamlit)");
      System.out.print("\nSelect an option [1-2]: ");
      
      java.util.Scanner scanner = new java.util.Scanner(System.in);
      try {
        choice = Integer.parseInt(scanner.nextLine().trim());
      } catch (Exception e) {
        choice = 1; // Default to GUI
      }
    }

    if (choice == 2) {
      LoggerUtil.getLogger().info("[STARTUP] Starting REST API for External Dashboard...");
      SimulationLauncher launcher = coreLauncher();
      launcher.preloadForDashboard();
      launcher.wireAndStartRestServer(8081);
      
      // Launch dashboard.html in the default browser
      try {
          java.io.File htmlFile = new java.io.File("dashboard.html");
          if (htmlFile.exists() && java.awt.Desktop.isDesktopSupported()) {
              java.awt.Desktop.getDesktop().browse(htmlFile.toURI());
          } else {
              LoggerUtil.getLogger().warning("Could not launch browser automatically. Please open dashboard.html manually.");
          }
      } catch (Exception e) {
          LoggerUtil.getLogger().warning("Failed to open browser: " + e.getMessage());
      }
    } else {
      // Option 1: Standard GUI only — no REST API server is started.
      LoggerUtil.getLogger().info("[STARTUP] Launching Standard GUI...");
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

  protected ScenarioConfig buildScenarioConfig() {
    return new ScenarioConfig(StringEnum.Learner.values(), null);
  }

  protected Engine.StateFactory buildStateFactory() {
    return PedSimCity::new;
  }

  protected Engine buildEngine() {
    return new Engine(buildStateFactory());
  }

  /** Returns a {@link SimulationLauncher} configured for core (non-night) mode. */
  protected SimulationLauncher coreLauncher() {
    return new SimulationLauncher(
        false,
        () -> new Engine(PedSimCity::new),
        new ScenarioConfig(StringEnum.Learner.values(), null),
        null);
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
}
