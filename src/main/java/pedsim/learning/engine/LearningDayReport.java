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
    logger.info(
        String.format(
            "learning day %d: known network (edges, mean; of them beyond the community's) learners"
                + " %.0f (%.0f) over %d, others %.0f (%.0f) over %d; learners' recognition:"
                + " local-landmark threshold %.2f at vividness %.2f (%d queries), easiness"
                + " threshold %.2f at vividness %.2f (%d queries); route efficiency (straight line"
                + " / walked) learners %.4f over %d trips, others %.4f over %d",
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
            (int) efficiencyOthers[1]));
  }

  /**
   * With shadow pairs, one line per level of starting knowledge: the legs a learner and its shadow
   * both planned, the learner's route length against the shadow's and both against the shortest
   * path, and the known network of each.
   */
  private static void logPairs(PedSimCity state, int day) {
    String[] levels = LearningPars.knowledgeLevels.split(",");
    double[][] totals = new double[levels.length][5];
    double[] known = new double[levels.length];
    double[] knownShadow = new double[levels.length];
    int[] agents = new int[levels.length];
    for (Agent agent : state.agentsList) {
      if (agent instanceof LearningAgent learning && learning.getShadow() != null) {
        int level = agent.agentID % levels.length;
        double[] drained = learning.drainPairTotals();
        for (int i = 0; i < 5; i++) {
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
                  + " path, known edges learner %.0f shadow %.0f",
              day,
              levels[level].trim(),
              agents[level],
              (int) t[0],
              100.0 * (t[1] / t[2] - 1.0),
              100.0 * (t[1] / t[3] - 1.0),
              100.0 * (t[2] / t[3] - 1.0),
              known[level] / agents[level],
              knownShadow[level] / agents[level]));
    }
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
