package pedsim.core.website;

import io.javelit.core.Jt;
import io.javelit.core.JtContainer;
import java.util.List;
import java.util.Map;
import java.util.logging.Logger;
import pedsim.core.engine.Engine;
import pedsim.core.engine.ScenarioConfig;
import pedsim.core.engine.SimulationStateStore;
import pedsim.core.parameters.Pars;
import pedsim.core.parameters.ParameterManager;
import pedsim.core.parameters.TimePars;
import pedsim.core.utilities.LoggerUtil;
import pedsim.core.utilities.StringEnum;

/**
 * Base Javelit dashboard page for PedSimCity simulations.
 *
 * <p>Renders the simulation control panel and live Leaflet map. The page
 * re-executes top-to-bottom on every user interaction, consistent with
 * Javelit's execution model.
 */
public class SimulationApp {

  private static final Logger logger = LoggerUtil.getLogger();

  /** Default map centre — Muenster. Auto-corrected from road MBR once roads load. */
  private static final double[] DEFAULT_CENTRE = {51.96, 7.62};
  private static final int      DEFAULT_ZOOM   = 14;

  // ----------------------------------------------------------------
  // Main page render
  // ----------------------------------------------------------------

  /**
   * Entry point passed to {@code Server.builder()} for the core/day simulation.
   * Uses a default {@link SimulationApp} instance with no extra controls.
   */
  public static void render() {
    render(new SimulationApp());
  }

  /**
   * Renders the full dashboard using the given app instance.
   *
   * @param app the simulation app instance (core or night variant).
   */
  public static void render(SimulationApp app) {
    // ---- Header ----
    Jt.title(app.getTitle()).use();
    Jt.divider("header-div").use();

    // ---- Two-column layout: sidebar (30%) | main (70%) ----
    var cols = Jt.columns(2).widths(List.of(0.3, 0.7)).use();
    JtContainer sidebar = cols.col(0);
    JtContainer main    = cols.col(1);

    // === SIDEBAR ===
    app.renderControlPanel(sidebar);

    // === MAIN PANEL ===
    renderMainPanel(main);
  }

  // ----------------------------------------------------------------
  // Sidebar: control panel
  // ----------------------------------------------------------------

  /**
   * Renders the simulation parameter form in the sidebar column.
   *
   * @param sidebar the Javelit container for the sidebar column.
   */
  protected void renderControlPanel(JtContainer sidebar) {
    Jt.subheader("Parameters").use(sidebar);

    // All inputs grouped in a form — simulation only starts on submit
    var form = Jt.form().use(sidebar);

    // City name dropdown — matches AWT Choice widget
    String cityName = Jt.selectbox("City Name",
        List.of("TorinoCentre", "Muenster")).use(form);

    // Numeric text fields — match AWT TextField widgets exactly
    Number days       = Jt.numberInput("Duration in days").value(7).use(form);
    Number population = Jt.numberInput("Actual Population").value(100000).use(form);
    String pctAgents  = Jt.textInput("% Represented by Agents").value("0.01").use(form);
    Number jobs       = Jt.numberInput("Jobs").value(1).use(form);

    // Delegate to subclass for any simulation-variant-specific extra controls
    renderExtraControls(form);

    boolean startClicked = Jt.formSubmitButton("Run Simulation").use(form);

    // Stop button — only shown while simulation is running
    if (SimulationStateStore.getInstance().running) {
      boolean stopClicked = Jt.button("Stop Simulation").use(sidebar);
      if (stopClicked)
        SimulationStateStore.getInstance().requestStop();
    }

    // Fire the simulation when the form is submitted and not already running
    if (startClicked && !SimulationStateStore.getInstance().running) {
      startSimulation(
          cityName,
          days.intValue(),
          population.intValue(),
          pctAgents,
          jobs.intValue()
      );
    }

    // Status feedback
    Jt.divider("status-div").use(sidebar);
    SimulationStateStore store = SimulationStateStore.getInstance();
    
    if (store.running || store.finished) {
      Jt.html("""
        <div id="live-stats" style="margin-bottom: 16px; font-family: system-ui, sans-serif;">
          <div style="display: flex; justify-content: space-between; margin-bottom: 8px;">
            <span style="color: #94a3b8; font-size: 12px;">TIME: <b id="side-time" style="color: #f8fafc;">%s</b></span>
            <span style="color: #94a3b8; font-size: 12px;">STEP: <b id="side-step" style="color: #f8fafc;">%d</b></span>
          </div>
          <div style="display: grid; grid-template-columns: 1fr 1fr 1fr; gap: 8px; text-align: center;">
            <div style="background: #1e293b; padding: 8px; border-radius: 8px; border: 1px solid #334155;">
              <div style="font-size: 18px;">🚶</div>
              <div id="side-walking" style="font-weight: 600; color: #f8fafc;">%d</div>
            </div>
            <div style="background: #1e293b; padding: 8px; border-radius: 8px; border: 1px solid #334155;">
              <div style="font-size: 18px;">🏠</div>
              <div id="side-home" style="font-weight: 600; color: #f8fafc;">%d</div>
            </div>
            <div style="background: #1e293b; padding: 8px; border-radius: 8px; border: 1px solid #334155;">
              <div style="font-size: 18px;">🎯</div>
              <div id="side-dest" style="font-weight: 600; color: #f8fafc;">%d</div>
            </div>
          </div>
        </div>
        <script>
          function updateSidebar() {
            fetch('http://localhost:%d/api/state')
              .then(r => r.json())
              .then(data => {
                document.getElementById('side-time').textContent = data.simulationTime || '—';
                document.getElementById('side-step').textContent = data.currentStep || '0';
                document.getElementById('side-walking').textContent = data.walkingCount || '0';
                document.getElementById('side-home').textContent = data.atHomeCount || '0';
                document.getElementById('side-dest').textContent = data.atDestCount || '0';
              }).catch(e => console.error('Sidebar update failed', e));
          }
          setInterval(updateSidebar, 1000);
        </script>
        """.formatted(
            store.simulationTime, store.currentStep, 
            store.walkingCount, store.atHomeCount, store.atDestCount,
            SimulationWebServer.API_PORT
        )).use(sidebar);
      Jt.divider("stats-sep").use(sidebar);
    }

    if (store.running)       Jt.success("Simulation running...").use(sidebar);
    else if (store.finished) Jt.info("Simulation finished.").use(sidebar);
    else                     Jt.info("Configure parameters and click Run.").use(sidebar);
  }

  // ----------------------------------------------------------------
  // Main panel: map
  // ----------------------------------------------------------------

  /**
   * Renders the Leaflet map block in the main column.
   *
   * @param main the Javelit container for the main column.
   */
  private static void renderMainPanel(JtContainer main) {
    double[] centre = deriveCentre();
    // Pass 0,0,0 as centre/zoom are now handled internally by the Canvas engine
    String mapHtml = MapHtmlBuilder.build(new double[]{0,0}, 0, SimulationWebServer.API_PORT);
    Jt.html(mapHtml).use(main);
  }

  // ----------------------------------------------------------------
  // Extension points for subclasses
  // ----------------------------------------------------------------

  /**
   * Returns the page title displayed in the dashboard header.
   * Subclasses override this to customise the title per simulation variant.
   *
   * @return the page title string.
   */
  protected String getTitle() {
    return "PedSimCity";
  }

  /**
   * Hook for subclasses to inject additional form controls.
   *
   * @param form the Javelit form container to add widgets to.
   */
  protected void renderExtraControls(JtContainer form) {
    // no-op — overridden by NightSimulationApp if needed
  }

  /**
   * Returns the {@link ScenarioConfig} for this simulation variant.
   */
  protected ScenarioConfig buildScenarioConfig() {
    return new ScenarioConfig(StringEnum.Learner.values(), null);
  }

  /**
   * Returns the {@link Engine.StateFactory} for this simulation variant.
   */
  protected Engine.StateFactory buildStateFactory() {
    return pedsim.core.engine.PedSimCity::new;
  }

  // ----------------------------------------------------------------
  // Simulation launch
  // ----------------------------------------------------------------

  private void startSimulation(String cityName, int days,
      int population, String pctAgents, int jobs) {

    SimulationStateStore store = SimulationStateStore.getInstance();
    store.reset();
    store.running = true;

    // Apply Pars fields — cityName, population, percentagePopulationAgent, jobs, durationDays
    ParameterManager.applyParams(
        Map.of(
            "cityName",                  cityName,
            "population",                String.valueOf(population),
            "percentagePopulationAgent", pctAgents,
            "jobs",                      String.valueOf(jobs),
            "durationDays",              String.valueOf(days)
        ),
        Pars.class
    );

    // Apply TimePars field — numberOfDays
    ParameterManager.applyParams(
        Map.of("numberOfDays", String.valueOf(days)),
        TimePars.class
    );

    ScenarioConfig      scenarioConfig = buildScenarioConfig();
    Engine.StateFactory stateFactory   = buildStateFactory();

    Thread simThread = new Thread(() -> {
      try {
        Engine engine = new Engine(stateFactory);
        engine.runJobs(scenarioConfig, Pars.parallel);
        store.finished = true;
      } catch (Exception ex) {
        logger.severe("Simulation error: " + ex.getMessage());
        ex.printStackTrace();
      } finally {
        store.running = false;
      }
    }, "sim-thread");

    simThread.setDaemon(true);
    simThread.start();
  }

  private static double[] deriveCentre() {
    try {
      var mbr = pedsim.core.engine.PedSimCity.roads.getMBR();
      if (mbr != null && !mbr.isNull()) {
        return new double[]{
            (mbr.getMinY() + mbr.getMaxY()) / 2.0,
            (mbr.getMinX() + mbr.getMaxX()) / 2.0
        };
      }
    } catch (Exception ignored) {}
    return DEFAULT_CENTRE;
  }
}
