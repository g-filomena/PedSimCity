package pedsim.cityimage.agents;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.ArrayList;
import java.util.EnumMap;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import org.javatuples.Pair;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.TestInstance;
import org.locationtech.jts.planargraph.DirectedEdge;
import pedsim.cityimage.engine.CityImageEngine;
import pedsim.cityimage.engine.PedSimCityImage;
import pedsim.cityimage.parameters.TestPars;
import pedsim.cityimage.utilities.StringEnum.Scenario;
import pedsim.core.agents.Agent;
import pedsim.core.engine.PedSimCity;
import pedsim.core.engine.ScenarioConfig;
import pedsim.core.parameters.Pars;
import pedsim.core.parameters.RouteChoicePars;
import sim.graph.NodeGraph;
import sim.routing.Route;

/**
 * What a route-choice model does on a real city, which no unit test on the model objects can say.
 *
 * <p>The defect these exist for is not a wrong answer but <b>a mechanism that runs without
 * effect</b>: a gate on a set that is permanently empty, a threshold that is never crossed, a
 * derived layer nothing fills. It has happened at least ten times in this repository — eight dark
 * gates on {@code agentKnown*} sets, {@code getWayfindingEasinessThreshold} returning 0 so no
 * on-route mark was ever inserted, {@code edgesWithinParks} with no writer — and every one of them
 * presented as a model that <i>had no effect</i> rather than a model that <i>never ran</i>. Nothing
 * in the fast suite can see the difference, because the difference is only visible against a graph.
 *
 * <p>Two checks, on one prepared city:
 *
 * <ul>
 *   <li><b>{@code ROAD_DISTANCE} is minimal.</b> With the perception error pinned to zero it is the
 *       true shortest path by construction, so any model that beats it is not routing the OD pair it
 *       was given. This is the check that makes a silent substitution impossible to miss.
 *   <li><b>A scenario that names an element differs from its sibling.</b> Two scenarios that agree
 *       on every OD pair are one code path wearing two names, which is exactly how region
 *       navigation, barrier sub-goals and on-route marks each spent months inert.
 * </ul>
 *
 * <p><b>{@code --perceptionErrorSD=0} is the whole point of the fixture.</b> At the default 0.10
 * every edge cost in every model carries a random multiplier, which produces roughly 0.43 edge
 * overlap between two runs of a model and <i>itself</i>; two claims were made and retracted on 15
 * September 2026 because that noise was read as signal. Pinned, a difference is the model.
 *
 * <p>Tagged {@code slow} — it imports and prepares a whole city — so {@code mvn test} does not run
 * it. Run it with {@code mvn test -Dgroups=slow}.
 */
@Tag("slow")
@TestInstance(TestInstance.Lifecycle.PER_CLASS)
class RouteChoiceOnACityTest {

  /**
   * The smallest bundled city that satisfies the one-city-one-folder rule and carries everything the
   * scenarios need: a dual graph for angular routing, barriers, and 6,867 buildings with
   * {@code lScore_sc} / {@code gScore_sc} plus 36,732 sight lines for the landmark models.
   * {@code Torino_centre} is smaller but its files are prefixed {@code TorinoCentre_}, which
   * {@code --cityName} cannot address.
   */
  private static final String CITY = "Muenster";

  /**
   * Enough OD pairs that "these two models never differ" is a statement about the models and not
   * about a lucky draw, and few enough that the whole class routes in seconds once the city is up.
   */
  private static final int OD_PAIRS = 40;

  /**
   * Metres of slack when comparing two route lengths. Floating-point summation over a few hundred
   * edges, not a modelling allowance: a model that genuinely beats the shortest path beats it by
   * much more than this.
   */
  private static final double LENGTH_TOLERANCE_M = 1e-6;

  /**
   * How many element-versus-sibling pairs {@value #CITY} can actually exercise, asserted rather than
   * discovered, so that a city quietly losing a layer turns this test red instead of green. It is
   * five: region and barrier navigation, in both their distance and angular forms, plus the two
   * combined.
   *
   * <p>Eight: region and barrier navigation in both their distance and angular forms, the two
   * combined, and the three landmark pairs. Muenster carries all of it - {@code lScore_sc},
   * {@code gScore_sc} and 36,732 sight lines on 6,867 buildings - so the landmark models are covered
   * too, which matters more than the rest: the on-route mark that no version of this code had ever
   * inserted lived there, and it stayed hidden for months because three differently-configured
   * landmark models agreeing to four decimal places was read as agreement rather than as absence.
   */
  private static final int ELEMENT_PAIRS_THIS_CITY_SUPPORTS = 8;

  private static PedSimCityImage city;
  private static Map<Scenario, CityImageAgent> agents;
  private static List<Pair<NodeGraph, NodeGraph>> odPairs;

  @BeforeAll
  void prepareCity() throws Exception {
    Pars.cityName = CITY;
    Pars.jobs = 1;
    Pars.exportHtmlDashboard = false;
    // The control this whole class depends on: every edge cost is then exactly its length.
    RouteChoicePars.perceptionErrorSD = 0.0;

    // "Testing Specific Route Choice Models" is the only design whose importer reads everything.
    // CityImageImport.importFiles() decides what the city IS from the design: the landmarks design
    // skips barriers, and the subdivisions design skips buildings AND sight lines - so running the
    // full scenario list under that design gives the landmark models no landmarks and they fall
    // back silently, which is the very defect this class exists to catch, staged by the test
    // itself.
    TestPars.stringMode = "Testing Specific Route Choice Models";
    TestPars.defineMode();
    // defineMode applies the design's own figures; a test wants its own.
    TestPars.numberTripsPerAgent = OD_PAIRS;
    TestPars.scenarios = Scenario.values();
    Pars.numAgents = TestPars.scenarios.length;
    Pars.jobs = 1;

    ScenarioConfig config = new ScenarioConfig(Scenario.values(), null);
    PreparedCity engine = new PreparedCity(config);
    engine.runJobs(config, false);

    city = engine.city;
    assertNotNull(city, "the city-image engine never reached its diagnostic hook");

    agents = new EnumMap<>(Scenario.class);
    for (Agent agent : city.getAgentsList()) {
      if (agent instanceof CityImageAgent cityImageAgent
          && cityImageAgent.getAgentScenario() instanceof Scenario scenario) {
        agents.put(scenario, cityImageAgent);
      }
    }
    assertFalse(agents.isEmpty(), "no city-image agents were built");

    odPairs = new ArrayList<>(agents.values().iterator().next().OD);
    assertFalse(odPairs.isEmpty(), "no OD pairs were generated");
  }

  /**
   * On identical OD pairs, is the distance baseline the shortest route?
   *
   * <p>If another model beats it, the baseline is not minimising distance and nothing else in a
   * comparison against it can be trusted. This is the one check that catches a model silently
   * serving something other than the route it was asked for — including the case the angular
   * fallback used to produce, where a model returned a shortest path under its own name.
   */
  @Test
  void roadDistanceIsMinimalOnEveryOdPair() {
    CityImageAgent baseline = agents.get(Scenario.ROAD_DISTANCE);
    assertNotNull(baseline, "ROAD_DISTANCE did not run");

    List<String> offences = new ArrayList<>();
    for (int trip = 0; trip < odPairs.size(); trip++) {
      double shortest = routeLength(baseline, trip);
      if (shortest <= 0.0) {
        continue; // an OD pair the baseline itself cannot route says nothing about the others
      }
      for (Map.Entry<Scenario, CityImageAgent> entry : agents.entrySet()) {
        if (entry.getKey() == Scenario.ROAD_DISTANCE) {
          continue;
        }
        double length = routeLength(entry.getValue(), trip);
        if (length > 0.0 && length < shortest - LENGTH_TOLERANCE_M) {
          offences.add(
              String.format(
                  "OD %d: %s routed %.1f m against ROAD_DISTANCE's %.1f m",
                  trip, entry.getKey(), length, shortest));
        }
      }
    }
    assertTrue(
        offences.isEmpty(),
        () ->
            "ROAD_DISTANCE is not minimal, so it is not the shortest path and no comparison "
                + "against it means anything:\n  "
                + String.join("\n  ", offences));
  }

  /**
   * A scenario that names a city-image element has to route differently from the scenario that does
   * not name it, on at least one OD pair out of {@value #OD_PAIRS}.
   *
   * <p>Byte-identical siblings are what the September 2026 audit kept finding, and the tell was
   * always read as agreement rather than as absence — three differently-configured landmark models
   * scoring 1.0740 to four decimal places on 255 ODs were not agreeing, they were one code path.
   *
   * <p>A pair whose data this city does not carry is skipped and named in the failure message, so
   * the test cannot pass by silently comparing nothing.
   */
  @Test
  void eachElementScenarioDiffersFromItsSibling() {
    Map<Scenario, Scenario> siblings = new LinkedHashMap<>();
    siblings.put(Scenario.REGION_DISTANCE, Scenario.ROAD_DISTANCE);
    siblings.put(Scenario.REGION_ANGULAR, Scenario.ANGULAR_CHANGE);
    siblings.put(Scenario.BARRIER_DISTANCE, Scenario.ROAD_DISTANCE);
    siblings.put(Scenario.BARRIER_ANGULAR, Scenario.ANGULAR_CHANGE);
    siblings.put(Scenario.REGION_BARRIER_DISTANCE, Scenario.REGION_DISTANCE);
    siblings.put(Scenario.LANDMARKS_DISTANCE, Scenario.ROAD_DISTANCE);
    siblings.put(Scenario.LOCAL_LANDMARKS_DISTANCE, Scenario.LANDMARKS_DISTANCE);
    siblings.put(Scenario.DISTANT_LANDMARKS_DISTANCE, Scenario.LANDMARKS_DISTANCE);

    List<String> identical = new ArrayList<>();
    List<String> compared = new ArrayList<>();
    List<String> skipped = new ArrayList<>();

    for (Map.Entry<Scenario, Scenario> entry : siblings.entrySet()) {
      Scenario element = entry.getKey();
      Scenario sibling = entry.getValue();
      String missing = whatThisCityLacksFor(element);
      if (missing != null) {
        skipped.add(element + " (" + missing + ")");
        continue;
      }
      CityImageAgent with = agents.get(element);
      CityImageAgent without = agents.get(sibling);
      if (with == null || without == null) {
        skipped.add(element + " (scenario did not run)");
        continue;
      }
      compared.add(element + " vs " + sibling);
      if (!differsOnAnyOdPair(with, without)) {
        identical.add(element + " is byte-identical to " + sibling + " on every OD pair");
      }
    }

    assertTrue(
        compared.size() >= ELEMENT_PAIRS_THIS_CITY_SUPPORTS,
        () ->
            "expected at least "
                + ELEMENT_PAIRS_THIS_CITY_SUPPORTS
                + " element scenarios to be exercisable on "
                + CITY
                + " but only "
                + compared.size()
                + " were, so this test now checks less than it used to. Either a data layer "
                + "stopped loading or a scenario stopped running. Compared: "
                + compared
                + "; skipped: "
                + skipped);
    assertTrue(
        identical.isEmpty(),
        () ->
            "a scenario that names an element routed exactly like the one that does not, which "
                + "means the element never reached the route:\n  "
                + String.join("\n  ", identical)
                + "\n(compared: "
                + compared
                + "; skipped for missing data: "
                + skipped
                + ")");
  }

  /** Why this city cannot exercise a scenario, or null when it can. */
  private static String whatThisCityLacksFor(Scenario scenario) {
    String name = scenario.name();
    if (name.contains("ANGULAR") && PedSimCity.dualNetwork.getNodes().isEmpty()) {
      return "no dual graph";
    }
    if (name.contains("REGION") && PedSimCity.regionsMap.size() < 2) {
      return "fewer than two regions";
    }
    if (name.contains("BARRIER") && PedSimCity.barriersMap.isEmpty()) {
      return "no barriers";
    }
    if (name.contains("LANDMARK") && !PedSimCity.landmarksLoaded) {
      // Either the city has no lScore_sc/gScore_sc on its buildings, or - far more likely - the
      // running test design did not load them. See CityImageImport.importFiles().
      return "no landmark scores loaded";
    }
    return null;
  }

  private static boolean differsOnAnyOdPair(CityImageAgent one, CityImageAgent other) {
    for (int trip = 0; trip < odPairs.size(); trip++) {
      if (!edgeIds(one, trip).equals(edgeIds(other, trip))) {
        return true;
      }
    }
    return false;
  }

  /** Routes {@code trip} with this agent's own model and returns the edges it used, in order. */
  private static List<Integer> edgeIds(CityImageAgent agent, int trip) {
    Route route = routeFor(agent, trip);
    List<Integer> ids = new ArrayList<>();
    if (route == null || route.directedEdgesSequence == null) {
      return ids;
    }
    for (DirectedEdge directedEdge : route.directedEdgesSequence) {
      ids.add(((sim.graph.EdgeGraph) directedEdge.getEdge()).getID());
    }
    return ids;
  }

  private static double routeLength(CityImageAgent agent, int trip) {
    Route route = routeFor(agent, trip);
    return route == null ? 0.0 : route.getLength();
  }

  private static Route routeFor(CityImageAgent agent, int trip) {
    Pair<NodeGraph, NodeGraph> od = agent.OD.get(trip);
    agent.originNode = od.getValue0();
    agent.destinationNode = od.getValue1();
    agent.planRoute();
    return agent.getRoute();
  }

  /**
   * A city-image engine that stops after preparing the city.
   *
   * <p>Uses {@code runDiagnosticsInstead()}, the hook core already offers for work that needs the
   * prepared city and no simulated days. Preparing the city by hand instead would be a second,
   * drifting copy of the import-and-prepare sequence, and a test that prepares a city differently
   * from a run is testing a city no run produces.
   */
  private static final class PreparedCity extends CityImageEngine {

    private final ScenarioConfig config;
    private PedSimCityImage city;

    private PreparedCity(ScenarioConfig config) {
      super(PedSimCityImage::new);
      this.config = config;
    }

    @Override
    protected boolean runDiagnosticsInstead() {
      city = (PedSimCityImage) stateFactory.create(baseSeed, 0, config);
      city.start();
      return true;
    }
  }
}
