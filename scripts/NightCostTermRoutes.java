import java.io.IOException;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Comparator;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Locale;
import java.util.Map;
import java.util.PriorityQueue;
import java.util.Set;
import org.locationtech.jts.geom.Coordinate;
import pedsim.core.cognition.cognitivemap.SharedCognitiveMap;
import pedsim.core.engine.Engine;
import pedsim.core.engine.PedSimCity;
import pedsim.core.engine.ScenarioConfig;
import pedsim.core.engine.SimulationLauncher;
import pedsim.core.engine.SimulationModule;
import pedsim.night.engine.Enclosure;
import pedsim.night.engine.EyesOnStreet;
import pedsim.night.engine.NightEngine;
import pedsim.night.engine.NightLighting;
import pedsim.night.engine.NightSimulationModule;
import pedsim.night.engine.PedSimCityNight;
import pedsim.night.parameters.NightPars;
import sim.graph.EdgeGraph;
import sim.graph.NodeGraph;
import ec.util.MersenneTwisterFast;

/**
 * One trip after dark, routed again as each term of the night cost is added: the shortest route,
 * then darkness, parks and rivers, enclosure and an empty street, for a vulnerable agent, and the
 * whole cost at the non-vulnerable weights. Written for the results site.
 *
 * <p>The cost of an edge is {@code NightRouteCost.factor}'s, term by term, at the {@link NightPars}
 * weights, for an agent that knows every street's measured lighting and sees no one on the street.
 * Searches the prepared night city without simulating a day (the engine's diagnostic hook), samples
 * origin-destination pairs near the city's centre, and keeps those in which every added term changes
 * the route.
 *
 * <pre>
 * java -Xmx12g -cp "target/classes;$(cat cp.txt)" scripts/NightCostTermRoutes.java \
 *   --cityName=Torino --out=cost_terms.json [--edges=cost_terms_edges.csv] [--hour=22] \
 *   [--pairs=4000] [--keep=12]
 * </pre>
 *
 * <p>{@code --edges} also writes every street's lux, park-or-water flag, enclosure and eyes at the
 * hour, which {@code render_cost_terms.py} draws beneath the routes.
 */
public final class NightCostTermRoutes {

  private static final String[] STEPS = {
    "shortest", "darkness", "parksRivers", "enclosure", "emptyStreet", "nonVulnerable"
  };

  private static Path out = Path.of("cost_terms.json");
  private static Path edgesOut = null;
  private static Path streetsOut = null;
  private static int hour = 22;
  private static int pairs = 4000;
  private static int keep = 12;
  private static double minMetres = 900;
  private static double maxMetres = 1600;
  private static double centreRadius = 3500;

  public static void main(String[] args) throws Exception {
    List<String> passed = new ArrayList<>();
    for (String arg : args) {
      String[] kv = arg.replaceFirst("^--", "").split("=", 2);
      switch (kv[0]) {
        case "out" -> out = Path.of(kv[1]);
        case "edges" -> edgesOut = Path.of(kv[1]);
        case "streets" -> streetsOut = Path.of(kv[1]);
        case "hour" -> hour = Integer.parseInt(kv[1]);
        case "pairs" -> pairs = Integer.parseInt(kv[1]);
        case "keep" -> keep = Integer.parseInt(kv[1]);
        default -> passed.add(arg);
      }
    }
    new SimulationLauncher(new Module()).headlessRun(passed.toArray(String[]::new));
  }

  /** The night module, with an engine that searches the prepared city and simulates nothing. */
  private static final class Module implements SimulationModule {
    private final NightSimulationModule night = NightSimulationModule.INSTANCE;

    @Override
    public String moduleId() {
      return night.moduleId();
    }

    @Override
    public Class<?>[] parameterClasses() {
      return night.parameterClasses();
    }

    @Override
    public void loadCityConfig(String cityName) {
      night.loadCityConfig(cityName);
    }

    @Override
    public void clearStaticData() {
      night.clearStaticData();
    }

    @Override
    public long populationForCity(String city) {
      return night.populationForCity(city);
    }

    @Override
    public ScenarioConfig scenarioConfig() {
      return night.scenarioConfig();
    }

    @Override
    public Engine createEngine() {
      return new NightEngine(PedSimCityNight::new) {
        @Override
        protected boolean runDiagnosticsInstead() {
          try {
            search();
          } catch (IOException e) {
            throw new RuntimeException(e);
          }
          return true;
        }
      };
    }
  }

  /** The multiplier of one edge with the first {@code terms} terms of the cost switched on. */
  private static double factor(EdgeGraph edge, int terms, boolean vulnerable) {
    double f = 1.0;
    if (terms >= 1) {
      double w = vulnerable ? NightPars.darknessWeightVulnerable : NightPars.darknessWeightNonVulnerable;
      // A typical agent: the middle of the range reassurance levels are drawn from.
      double reassurance = (NightPars.minReassuranceLux + NightPars.maxReassuranceLux) / 2.0;
      f *= 1.0 + w * NightLighting.darkness(NightLighting.measuredLux(edge), reassurance);
    }
    if (terms >= 2 && SharedCognitiveMap.getEdgesWithinParksOrAlongWater().contains(edge)) {
      f *= 1.0 + (vulnerable ? NightPars.parkWaterWeightVulnerable : NightPars.parkWaterWeightNonVulnerable);
    }
    if (terms >= 3) {
      double w = vulnerable ? NightPars.enclosureWeightVulnerable : NightPars.enclosureWeightNonVulnerable;
      f *= 1.0 + w * Enclosure.of(edge);
    }
    if (terms >= 4) {
      double w = vulnerable ? NightPars.emptyStreetWeightVulnerable : NightPars.emptyStreetWeightNonVulnerable;
      f *= 1.0 + w * (1.0 - EyesOnStreet.at(edge, hour));
    }
    return f;
  }

  /** Dijkstra from one node to another; the edges in walking order, or null. */
  private static List<EdgeGraph> route(NodeGraph from, NodeGraph to, int terms, boolean vulnerable) {
    Map<Integer, Double> cost = new HashMap<>();
    Map<Integer, EdgeGraph> via = new HashMap<>();
    Set<Integer> done = new HashSet<>();
    PriorityQueue<double[]> queue =
        new PriorityQueue<>(Comparator.<double[]>comparingDouble(e -> e[0]).thenComparingDouble(e -> e[1]));
    cost.put(from.getID(), 0.0);
    queue.add(new double[] {0.0, from.getID()});
    while (!queue.isEmpty()) {
      double[] head = queue.poll();
      int id = (int) head[1];
      if (!done.add(id)) {
        continue;
      }
      if (id == to.getID()) {
        break;
      }
      NodeGraph node = PedSimCity.nodesMap.get(id);
      for (EdgeGraph edge : node.getEdges()) {
        NodeGraph next = edge.getOtherNode(node);
        double c = head[0] + edge.getLength() * factor(edge, terms, vulnerable);
        if (c < cost.getOrDefault(next.getID(), Double.POSITIVE_INFINITY)) {
          cost.put(next.getID(), c);
          via.put(next.getID(), edge);
          queue.add(new double[] {c, next.getID()});
        }
      }
    }
    if (!via.containsKey(to.getID())) {
      return null;
    }
    List<EdgeGraph> path = new ArrayList<>();
    NodeGraph node = to;
    while (node.getID() != from.getID()) {
      EdgeGraph edge = via.get(node.getID());
      path.add(0, edge);
      node = edge.getOtherNode(node);
    }
    return path;
  }

  private static Set<Integer> ids(List<EdgeGraph> path) {
    Set<Integer> s = new HashSet<>();
    path.forEach(e -> s.add(e.getID()));
    return s;
  }

  /** Share of the longer route's metres not shared with the other. */
  private static double difference(List<EdgeGraph> a, List<EdgeGraph> b) {
    Set<Integer> shared = ids(a);
    shared.retainAll(ids(b));
    double sharedM = 0, aM = 0, bM = 0;
    for (EdgeGraph e : a) {
      aM += e.getLength();
      if (shared.contains(e.getID())) sharedM += e.getLength();
    }
    for (EdgeGraph e : b) bM += e.getLength();
    return 1.0 - sharedM / Math.max(aM, bM);
  }

  private static void search() throws IOException {
    List<NodeGraph> nodes = new ArrayList<>(PedSimCity.nodesMap.values());
    nodes.sort(Comparator.comparingInt(NodeGraph::getID));
    double cx = nodes.stream().mapToDouble(n -> n.getCoordinate().x).average().orElse(0);
    double cy = nodes.stream().mapToDouble(n -> n.getCoordinate().y).average().orElse(0);
    Coordinate centre = new Coordinate(cx, cy);
    List<NodeGraph> central =
        nodes.stream().filter(n -> n.getCoordinate().distance(centre) <= centreRadius).toList();
    MersenneTwisterFast random = new MersenneTwisterFast(20261009L);

    List<String> kept = new ArrayList<>();
    List<Double> scores = new ArrayList<>();
    int tried = 0;
    for (int p = 0; p < pairs; p++) {
      NodeGraph o = central.get(random.nextInt(central.size()));
      NodeGraph d = central.get(random.nextInt(central.size()));
      double crow = o.getCoordinate().distance(d.getCoordinate());
      if (crow < minMetres || crow > maxMetres) continue;
      tried++;
      List<List<EdgeGraph>> routes = new ArrayList<>();
      for (int t = 0; t <= 4; t++) routes.add(route(o, d, t, true));
      routes.add(route(o, d, 4, false));
      if (routes.stream().anyMatch(r -> r == null || r.isEmpty())) continue;
      double score = 0;
      boolean everyStepChanges = true;
      for (int t = 1; t <= 4; t++) {
        double diff = difference(routes.get(t - 1), routes.get(t));
        if (diff < 0.08) everyStepChanges = false;
        score += Math.min(diff, 0.4);
      }
      double menVsWomen = difference(routes.get(4), routes.get(5));
      double detour = length(routes.get(4)) / length(routes.get(0));
      if (!everyStepChanges || menVsWomen < 0.08 || detour > 1.3) continue;
      score += Math.min(menVsWomen, 0.4);
      kept.add(candidate(o, d, routes));
      scores.add(score);
    }
    Integer[] order = new Integer[kept.size()];
    for (int i = 0; i < order.length; i++) order[i] = i;
    Arrays.sort(order, Comparator.comparingDouble(i -> -scores.get(i)));
    StringBuilder json = new StringBuilder();
    json.append(String.format(Locale.ROOT,
        "{\"city\":\"%s\",\"hour\":%d,\"steps\":[%s],\"weights\":{\"vulnerable\":[%s,%s,%s,%s],"
            + "\"nonVulnerable\":[%s,%s,%s,%s]},\"tried\":%d,\"candidates\":[",
        pedsim.core.parameters.Pars.cityName, hour, quoted(STEPS),
        NightPars.darknessWeightVulnerable, NightPars.parkWaterWeightVulnerable,
        NightPars.enclosureWeightVulnerable, NightPars.emptyStreetWeightVulnerable,
        NightPars.darknessWeightNonVulnerable, NightPars.parkWaterWeightNonVulnerable,
        NightPars.enclosureWeightNonVulnerable, NightPars.emptyStreetWeightNonVulnerable, tried));
    for (int i = 0; i < Math.min(keep, order.length); i++) {
      if (i > 0) json.append(',');
      json.append(kept.get(order[i]).replaceFirst("\\{", String.format(Locale.ROOT, "{\"score\":%.4f,", scores.get(order[i]))));
    }
    json.append("]}");
    Files.writeString(out, json, StandardCharsets.UTF_8);
    if (edgesOut != null) {
      StringBuilder csv = new StringBuilder("edgeID,lux,park_or_water,enclosure,eyes\n");
      List<EdgeGraph> all = new ArrayList<>(PedSimCity.edgesMap.values());
      all.sort(Comparator.comparingInt(EdgeGraph::getID));
      for (EdgeGraph e : all) {
        csv.append(String.format(Locale.ROOT, "%d,%.2f,%d,%.3f,%.3f%n", e.getID(),
            NightLighting.measuredLux(e),
            SharedCognitiveMap.getEdgesWithinParksOrAlongWater().contains(e) ? 1 : 0,
            Enclosure.of(e), EyesOnStreet.at(e, hour)));
      }
      Files.writeString(edgesOut, csv, StandardCharsets.UTF_8);
    }
    if (streetsOut != null) {
      writeStreets();
    }
    System.out.printf(Locale.ROOT, "%d pairs in range, %d where every term changes the route -> %s%n",
        tried, kept.size(), out.toAbsolutePath());
  }

  /**
   * Every street's terms, for the site's street map: lux, park and water flags, the smallest isovist
   * (-1 when none was measured), enclosure, and the venues open along it at each hour, recovered
   * from its eyes ({@code EyesOnStreet.eyes} inverted; -1 when the eyes saturate).
   */
  private static void writeStreets() throws IOException {
    StringBuilder csv = new StringBuilder("edgeID,lux,park,water,iso_min,enclosure");
    for (int h = 0; h < 24; h++) csv.append(",open").append(h);
    csv.append('\n');
    List<EdgeGraph> all = new ArrayList<>(PedSimCity.edgesMap.values());
    all.sort(Comparator.comparingInt(EdgeGraph::getID));
    Set<EdgeGraph> parks = SharedCognitiveMap.getEdgesWithinParks();
    Set<EdgeGraph> water = SharedCognitiveMap.getEdgesAlongWater();
    for (EdgeGraph e : all) {
      Double iso = PedSimCityNight.streetIsovistAreaMin.get(e.getID());
      csv.append(String.format(Locale.ROOT, "%d,%.2f,%d,%d,%.0f,%.3f", e.getID(),
          NightLighting.measuredLux(e), parks.contains(e) ? 1 : 0, water.contains(e) ? 1 : 0,
          iso == null ? -1.0 : iso, Enclosure.of(e)));
      double perVenue = NightPars.eyesSaturationPer100m * Math.max(e.getLength(), 50.0) / 100.0;
      for (int h = 0; h < 24; h++) {
        double eyes = EyesOnStreet.at(e, h);
        long open = eyes >= 1.0 ? -1 : Math.round(-Math.log1p(-eyes) * perVenue);
        csv.append(',').append(open);
      }
      csv.append('\n');
    }
    Files.writeString(streetsOut, csv, StandardCharsets.UTF_8);
  }

  private static double length(List<EdgeGraph> path) {
    return path.stream().mapToDouble(EdgeGraph::getLength).sum();
  }

  private static String quoted(String[] values) {
    StringBuilder s = new StringBuilder();
    for (int i = 0; i < values.length; i++) s.append(i > 0 ? "," : "").append('"').append(values[i]).append('"');
    return s.toString();
  }

  /** One pair: its routes with what each walks through, and every edge any of them uses. */
  private static String candidate(NodeGraph o, NodeGraph d, List<List<EdgeGraph>> routes) {
    StringBuilder s = new StringBuilder("{");
    s.append(String.format(Locale.ROOT, "\"origin\":%d,\"destination\":%d,\"routes\":[", o.getID(), d.getID()));
    Map<Integer, EdgeGraph> used = new HashMap<>();
    for (int r = 0; r < routes.size(); r++) {
      List<EdgeGraph> path = routes.get(r);
      double m = 0, dim = 0, park = 0, enclosed = 0, empty = 0;
      for (EdgeGraph e : path) {
        used.put(e.getID(), e);
        double len = e.getLength();
        m += len;
        if (NightLighting.measuredLux(e) < 10.0) dim += len;
        if (SharedCognitiveMap.getEdgesWithinParksOrAlongWater().contains(e)) park += len;
        if (Enclosure.of(e) > 0.0) enclosed += len;
        if (EyesOnStreet.at(e, hour) < 0.2) empty += len;
      }
      s.append(r > 0 ? "," : "");
      s.append(String.format(Locale.ROOT,
          "{\"step\":\"%s\",\"metres\":%.1f,\"under10lux\":%.1f,\"parksRivers\":%.1f,"
              + "\"enclosed\":%.1f,\"empty\":%.1f,\"edges\":[",
          STEPS[r], m, dim, park, enclosed, empty));
      for (int i = 0; i < path.size(); i++) s.append(i > 0 ? "," : "").append(path.get(i).getID());
      s.append("]}");
    }
    s.append("],\"edges\":{");
    List<Integer> keys = new ArrayList<>(used.keySet());
    keys.sort(null);
    for (int i = 0; i < keys.size(); i++) {
      EdgeGraph e = used.get(keys.get(i));
      s.append(i > 0 ? "," : "");
      s.append(String.format(Locale.ROOT,
          "\"%d\":{\"lux\":%.2f,\"park\":%b,\"enclosure\":%.3f,\"eyes\":%.3f}",
          e.getID(), NightLighting.measuredLux(e),
          SharedCognitiveMap.getEdgesWithinParksOrAlongWater().contains(e),
          Enclosure.of(e), EyesOnStreet.at(e, hour)));
    }
    s.append("}}");
    return s.toString();
  }
}
