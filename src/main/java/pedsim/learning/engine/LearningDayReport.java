package pedsim.learning.engine;

import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Set;
import java.util.logging.Logger;
import pedsim.core.agents.Agent;
import pedsim.core.cognition.cognitivemap.SharedCognitiveMap;
import pedsim.core.engine.PedSimCity;
import pedsim.core.utilities.LoggerUtil;
import pedsim.learning.agents.LearningAgent;
import pedsim.learning.cognitivemap.MemoryProfile;
import pedsim.learning.cognitivemap.RecognitionStats;
import pedsim.learning.parameters.LearningPars;
import sim.graph.EdgeGraph;

/**
 * One log line per simulated day with what the learning contrast measures: how large the network
 * the learners route over is, against the agents that never learn, and what recognition saw.
 * Agents whose map has not formed yet are left out; the line says how many each mean is over.
 */
final class LearningDayReport {

  private static final Logger logger = LoggerUtil.getLogger();

  private LearningDayReport() {}

  static void log(PedSimCity state, int day) {
    if (LearningPars.shadowPairs) {
      logPairs(state, day);
    }
    List<LearningAgent> learners = new ArrayList<>();
    List<LearningAgent> others = new ArrayList<>();
    for (Agent agent : state.agentsList) {
      if (agent instanceof LearningAgent learning) {
        if (!learning.getCognitiveMap().individualised) {
          continue; // no map yet: it forms on the agent's first activity
        }
        (learning.isLearner() ? learners : others).add(learning);
      }
    }
    RecognitionStats local = new RecognitionStats();
    RecognitionStats easiness = new RecognitionStats();
    for (LearningAgent learner : learners) {
      local.add(learner.getLearningCognitiveMap().drainLocalLandmarkStats());
      easiness.add(learner.getLearningCognitiveMap().drainEasinessStats());
    }
    double[] efficiencyLearners = new double[2];
    double[] efficiencyOthers = new double[2];
    for (Agent agent : state.agentsList) {
      if (agent instanceof LearningAgent learning) {
        double[] drained = learning.drainEfficiency();
        double[] into = learning.isLearner() ? efficiencyLearners : efficiencyOthers;
        into[0] += drained[0];
        into[1] += drained[1];
      }
    }
    int[] profiles = new int[MemoryProfile.values().length];
    for (LearningAgent learner : learners) {
      profiles[learner.memoryProfile().ordinal()]++;
    }
    logGroups(learners, day);
    logGrowth(learners, day);
    LearningSnapshots.write(learners, day);
    logger.info(
        String.format(
            "learning day %d: known network (edges, mean; of them beyond the community's) learners"
                + " %.0f (%.0f) over %d, others %.0f (%.0f) over %d; learners' recognition:"
                + " local-landmark threshold %.2f at vividness %.2f (%d queries), easiness"
                + " threshold %.2f at vividness %.2f (%d queries); route efficiency (straight line"
                + " / walked) learners %.4f over %d trips, others %.4f over %d; learners by memory"
                + " profile (integrators / non-integrators / imprecise) %d / %d / %d",
            day,
            meanKnownEdges(learners, false),
            meanKnownEdges(learners, true),
            learners.size(),
            meanKnownEdges(others, false),
            meanKnownEdges(others, true),
            others.size(),
            local.meanThreshold(),
            local.meanVividness(),
            local.queries(),
            easiness.meanThreshold(),
            easiness.meanVividness(),
            easiness.queries(),
            efficiencyLearners[0] / efficiencyLearners[1],
            (int) efficiencyLearners[1],
            efficiencyOthers[0] / efficiencyOthers[1],
            (int) efficiencyOthers[1],
            profiles[MemoryProfile.INTEGRATOR.ordinal()],
            profiles[MemoryProfile.NON_INTEGRATOR.ordinal()],
            profiles[MemoryProfile.IMPRECISE.ordinal()]));
  }

  /**
   * With shadow pairs, one line per level of starting knowledge: the legs a learner and its shadow
   * both planned, the learner's route length against the shadow's and both against the shortest
   * path, and the known network of each.
   */
  private static void logPairs(PedSimCity state, int day) {
    String[] levels = LearningPars.knowledgeLevels.split(",");
    double[][] totals = new double[levels.length][13];
    double[] known = new double[levels.length];
    double[] knownShadow = new double[levels.length];
    int[] agents = new int[levels.length];
    for (Agent agent : state.agentsList) {
      if (agent instanceof LearningAgent learning && learning.getShadow() != null) {
        int level = agent.agentID % levels.length;
        double[] drained = learning.drainPairTotals();
        for (int i = 0; i < drained.length; i++) {
          totals[level][i] += drained[i];
        }
        if (learning.getCognitiveMap().individualised) {
          known[level] += learning.getCognitiveMap().getEdgesInKnownNetwork().size();
          knownShadow[level] +=
              learning.getShadow().getCognitiveMap().getEdgesInKnownNetwork().size();
          agents[level]++;
        }
      }
    }
    for (int level = 0; level < levels.length; level++) {
      double[] t = totals[level];
      logger.info(
          String.format(
              "learning pairs day %d: level %s over %d pairs, %d legs: learner route %.2f%% longer"
                  + " than its shadow's, learner %.2f%% and shadow %.2f%% longer than the shortest"
                  + " path, known edges learner %.0f shadow %.0f, share of the route on main roads"
                  + " learner %.1f%% shadow %.1f%%, familiarity of the route: learner %.1f%%"
                  + " (%.1f%% of its metres on unfamiliar streets, %.1f%% of legs with one) shadow"
                  + " %.1f%% (%.1f%%, %.1f%%)",
              day,
              levels[level].trim(),
              agents[level],
              (int) t[0],
              100.0 * (t[1] / t[2] - 1.0),
              100.0 * (t[1] / t[3] - 1.0),
              100.0 * (t[2] / t[3] - 1.0),
              known[level] / agents[level],
              knownShadow[level] / agents[level],
              100.0 * t[5] / t[1],
              100.0 * t[6] / t[2],
              100.0 * t[7] / t[1],
              100.0 * t[8] / t[1],
              100.0 * t[11] / t[0],
              100.0 * t[9] / t[2],
              100.0 * t[10] / t[2],
              100.0 * t[12] / t[0]));
    }
  }

  /**
   * One line per day with, for each memory profile, how many learners it holds and the mean size of
   * their known network and of the part the community does not know.
   */
  private static void logGroups(List<LearningAgent> learners, int day) {
    String[] levels = LearningPars.knowledgeLevels.split(",");
    StringBuilder line = new StringBuilder("learning groups day " + day + ":");
    for (MemoryProfile profile : MemoryProfile.values()) {
      List<LearningAgent> group =
          learners.stream().filter(agent -> agent.memoryProfile() == profile).toList();
      line.append(
          String.format(
              " %s %d learners, known edges %.0f (beyond the community's %.0f) [by level:",
              profile, group.size(), meanKnownEdges(group, false), meanKnownEdges(group, true)));
      for (int level = 0; level < levels.length; level++) {
        final int levelIndex = level;
        List<LearningAgent> sub =
            group.stream().filter(agent -> agent.agentID % levels.length == levelIndex).toList();
        line.append(
            String.format(
                " %s: %d learners, %.0f;",
                levels[level].trim(), sub.size(), meanKnownEdges(sub, true)));
      }
      line.append("];");
    }
    logger.info(line.toString());
    if (day == 1 || day % 15 == 0) {
      logAgents(learners, day);
    }
  }

  /**
   * Every learner on one line, as {@code agentID:profile:level index:streets known beyond the
   * community's}, so a group's spread can be read and not only its mean.
   */
  private static void logAgents(List<LearningAgent> learners, int day) {
    String[] levels = LearningPars.knowledgeLevels.split(",");
    Set<EdgeGraph> community = new HashSet<>(SharedCognitiveMap.getCommunityKnownEdges());
    StringBuilder line = new StringBuilder("learning agents day " + day + ":");
    for (LearningAgent learner : learners) {
      long beyond =
          learner.getCognitiveMap().getEdgesInKnownNetwork().stream()
              .filter(edge -> !community.contains(edge))
              .count();
      line.append(
          String.format(
              " %d:%s:%s:%d",
              learner.agentID,
              learner.memoryProfile(),
              levels[learner.agentID % levels.length].trim(),
              beyond));
    }
    logger.info(line.toString());
  }

  /**
   * Where what the learners know beyond the community's network lies: the share of those streets
   * within 300 m of an anchor, and their mean distance from the nearest anchor.
   */
  private static void logGrowth(List<LearningAgent> learners, int day) {
    Set<EdgeGraph> community = new HashSet<>(SharedCognitiveMap.getCommunityKnownEdges());
    double streets = 0.0;
    double distance = 0.0;
    double within300 = 0.0;
    for (LearningAgent learner : learners) {
      var anchors = learner.getLearningCognitiveMap().getAnchors();
      if (anchors.isEmpty()) {
        continue;
      }
      for (EdgeGraph edge : learner.getCognitiveMap().getEdgesInKnownNetwork()) {
        if (community.contains(edge)) {
          continue;
        }
        var middle = edge.getLine().getCentroid().getCoordinate();
        double nearest = Double.MAX_VALUE;
        for (var anchor : anchors) {
          nearest = Math.min(nearest, middle.distance(anchor.getCoordinate()));
        }
        streets++;
        distance += nearest;
        if (nearest <= 300.0) {
          within300++;
        }
      }
    }
    logger.info(
        String.format(
            "learning growth day %d: streets known beyond the community's, all learners %.0f;"
                + " within 300 m of an anchor %.1f%%; mean"
                + " distance from the nearest anchor %.0f m",
            day, streets, 100.0 * within300 / streets, distance / streets));
  }

  /** The mean size of the agents' known networks, or of the part the community does not know. */
  private static double meanKnownEdges(List<LearningAgent> agents, boolean beyondCommunity) {
    if (agents.isEmpty()) {
      return Double.NaN;
    }
    Set<EdgeGraph> community = new HashSet<>(SharedCognitiveMap.getCommunityKnownEdges());
    return agents.stream()
        .mapToLong(
            agent ->
                agent.getCognitiveMap().getEdgesInKnownNetwork().stream()
                    .filter(edge -> !beyondCommunity || !community.contains(edge))
                    .count())
        .average()
        .orElse(Double.NaN);
  }
}
