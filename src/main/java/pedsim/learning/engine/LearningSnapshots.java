package pedsim.learning.engine;

import java.io.IOException;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.nio.file.StandardOpenOption;
import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Set;
import java.util.logging.Logger;
import pedsim.activity.agents.ActivityPurpose;
import pedsim.activity.engine.PedSimCityActivity;
import pedsim.activity.parameters.ActivityPars;
import pedsim.core.cognition.cognitivemap.SharedCognitiveMap;
import pedsim.core.engine.Exporter;
import pedsim.core.utilities.LoggerUtil;
import pedsim.learning.agents.LearningAgent;
import pedsim.learning.cognitivemap.CognitiveMap;
import pedsim.learning.parameters.LearningPars;
import sim.graph.EdgeGraph;
import sim.graph.NodeGraph;

/**
 * Writes each learner's activity space on the days {@link LearningPars#snapshotDays} names, under
 * {@code snapshots/} in the run's output folder:
 *
 * <ul>
 *   <li>{@code agents.csv}: per learner and day, the streets it knows (and those beyond the
 *       community's, in km), and for each purpose how many opportunities lie within the choice
 *       radius of home, how many of those it is aware of, and how many different places it has
 *       chosen;
 *   <li>{@code edges.csv}: per learner and day, the IDs of the streets it knows;
 *   <li>{@code homes.csv}: the learner's home, once;
 *   <li>{@code anchors.csv}: the places its cognitive map is anchored on (home, the workplace if it
 *       has one, the places its persona goes to), once.
 * </ul>
 *
 * An opportunity is a node holding something for the purpose; the agent is aware of it as {@link
 * CognitiveMap#isAwareOf} says.
 */
final class LearningSnapshots {

  private static final Logger logger = LoggerUtil.getLogger();

  private LearningSnapshots() {}

  static boolean due(int day) {
    for (String part : LearningPars.snapshotDays.split(",")) {
      if (!part.isBlank() && Integer.parseInt(part.trim()) == day) {
        return true;
      }
    }
    return false;
  }

  static void write(List<LearningAgent> learners, int day) {
    if (!due(day)) {
      return;
    }
    try {
      Path dir = Paths.get(Exporter.outputDirectory("PedSimCityLearning"), "snapshots");
      Files.createDirectories(dir);
      List<ActivityPurpose> purposes = purposes();
      Path agents = dir.resolve("agents.csv");
      if (!Files.exists(agents)) {
        StringBuilder header =
            new StringBuilder("day,agentID,profile,level,known_edges,beyond_community,known_km");
        for (ActivityPurpose purpose : purposes) {
          header.append(String.format(",%1$s_potential,%1$s_aware,%1$s_chosen", purpose));
        }
        append(agents, header.append('\n').toString());
      }
      Path homes = dir.resolve("homes.csv");
      boolean writeHomes = !Files.exists(homes);
      if (writeHomes) {
        append(homes, "agentID,x,y\n");
      }
      String[] levels = LearningPars.knowledgeLevels.split(",");
      Set<EdgeGraph> community = new HashSet<>(SharedCognitiveMap.getCommunityKnownEdges());
      StringBuilder rows = new StringBuilder();
      StringBuilder edgeRows = new StringBuilder();
      StringBuilder homeRows = new StringBuilder();
      StringBuilder anchorRows = new StringBuilder();
      for (LearningAgent learner : learners) {
        CognitiveMap map = learner.getLearningCognitiveMap();
        var known = learner.getCognitiveMap().getEdgesInKnownNetwork();
        long beyond = known.stream().filter(edge -> !community.contains(edge)).count();
        double metres = known.stream().mapToDouble(edge -> edge.getLine().getLength()).sum();
        rows.append(
            String.format(
                "%d,%d,%s,%s,%d,%d,%.2f",
                day,
                learner.agentID,
                learner.memoryProfile(),
                levels[learner.agentID % levels.length].trim(),
                known.size(),
                beyond,
                metres / 1000.0));
        var home = learner.homeNode.getCoordinate();
        for (ActivityPurpose purpose : purposes) {
          int potential = 0;
          int aware = 0;
          for (NodeGraph node : PedSimCityActivity.nodesPurposeWeight.get(purpose).keySet()) {
            if (PedSimCityActivity.nodesPurposeWeight.get(purpose).get(node) <= 0.0
                || node.getCoordinate().distance(home) > ActivityPars.choiceSetRadiusMetres) {
              continue;
            }
            potential++;
            if (map.isAwareOf(node)) {
              aware++;
            }
          }
          rows.append(
              String.format(",%d,%d,%d", potential, aware, learner.destinationsChosen(purpose)));
        }
        rows.append('\n');
        List<String> ids = new ArrayList<>(known.size());
        for (EdgeGraph edge : known) {
          ids.add(Integer.toString(edge.getID()));
        }
        edgeRows
            .append(day)
            .append(',')
            .append(learner.agentID)
            .append(',')
            .append(String.join(" ", ids))
            .append('\n');
        if (writeHomes) {
          homeRows.append(String.format("%d,%.2f,%.2f%n", learner.agentID, home.x, home.y));
          for (NodeGraph anchor : learner.getLearningCognitiveMap().getAnchors()) {
            String kind =
                anchor == learner.homeNode
                    ? "home"
                    : anchor == learner.workNode ? "work" : "anchor";
            anchorRows.append(
                String.format(
                    "%d,%s,%.2f,%.2f%n",
                    learner.agentID, kind, anchor.getCoordinate().x, anchor.getCoordinate().y));
          }
        }
      }
      append(agents, rows.toString());
      Path edges = dir.resolve("edges.csv");
      if (!Files.exists(edges)) {
        append(edges, "day,agentID,edges\n");
      }
      append(edges, edgeRows.toString());
      if (writeHomes) {
        append(homes, homeRows.toString());
        append(dir.resolve("anchors.csv"), "agentID,kind,x,y\n" + anchorRows);
      }
      logger.info(
          "learning snapshot day " + day + ": " + learners.size() + " learners written to " + dir);
    } catch (IOException e) {
      throw new RuntimeException(e);
    }
  }

  /** The purposes the city has opportunities for. */
  private static List<ActivityPurpose> purposes() {
    List<ActivityPurpose> found = new ArrayList<>();
    for (ActivityPurpose purpose : ActivityPurpose.values()) {
      Map<NodeGraph, Double> weights = PedSimCityActivity.nodesPurposeWeight.get(purpose);
      if (weights != null && !weights.isEmpty()) {
        found.add(purpose);
      }
    }
    return found;
  }

  private static void append(Path file, String text) throws IOException {
    Files.writeString(
        file, text, StandardCharsets.UTF_8, StandardOpenOption.CREATE, StandardOpenOption.APPEND);
  }
}
