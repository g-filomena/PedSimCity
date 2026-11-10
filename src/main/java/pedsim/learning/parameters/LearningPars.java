package pedsim.learning.parameters;

public class LearningPars {

  /**
   * Half-life, in days, of a cell's vividness for an agent whose spatial ability is 1. An agent with
   * ability 0 forgets twice as fast. See {@code IncrementalLearning.computeDecayFactor}.
   */
  public static double halfLife = 14;

  /**
   * The least strength, in walks, a cell needs to be part of the known space. A cell walked once is
   * known for one half-life at this value; a cell walked {@code n} times, for {@code 1 + log2(n)}.
   */
  public static double memoryThreshold = 0.5;

  /**
   * The share of a walk's weight that does not fade. Menzemer et al. fit route-knowledge retention
   * with an asymptote: a floor of 0.28 under a starting level of 0.88, a share of 0.32; the floor's
   * 95% interval, 0.01-0.84, leaves the share not identified, and their task was one exposure in a
   * building. 0 makes every walk fade to nothing. Only the first walk to reach a cell on a given
   * day adds to this share.
   */
  public static double consolidatedShare = 0.32;

  /**
   * Half-life, in days, of the consolidated part of a cell's memory for an agent whose spatial
   * ability is 1, with the same ability scaling as {@link #halfLife}. 0 means it never fades. Our
   * own assumption: Menzemer et al. follow retention for up to 12 months and see a floor, so what
   * happens after that is not observed.
   */
  public static double consolidatedHalfLife = 365;

  /**
   * How much a meaningful walk counts for more than one walk: a route with meaningfulness {@code m}
   * in [0, 1] adds {@code 1 + meaningfulnessGain * m} walks. Our own assumption, with no source; 0
   * turns meaningfulness off.
   */
  public static double meaningfulnessGain = 0.5;

  /** Side, in metres, of a vividness-grid cell. */
  public static double cellSize = 5.0;

  /**
   * Radius, in metres, of the buffers sampled around an origin, a destination and along the line
   * between them when measuring how vividly an agent knows the space between two places.
   */
  public static double RouteVividnessRadius = 400;

  /** What makes a learner prefer streets it has used: see {@code RouteHabit}. */
  public static pedsim.learning.habit.HabitMode habitMode = pedsim.learning.habit.HabitMode.NONE;

  /**
   * With {@code REMEMBERED}: the cost discount of a street remembered as vividly as possible. Our
   * own assumption, with no source.
   */
  public static double habitGain = 0.3;

  /**
   * With {@code REWARDED}: the cost discount per unit of a street's value, a value being the
   * relative gain in efficiency of the routes through it. Our own assumption, with no source.
   */
  public static double rewardGain = 2.0;

  /** With {@code REWARDED}: the share of the gap to a trip's gain that a street's value closes. */
  public static double rewardRate = 0.1;

  /**
   * The most days a forgotten cell may stay in the known network: the cognitive map is rebuilt from
   * the grid at most this often when fading alone has changed it. A walk rebuilds it at once.
   */
  public static double rebuildEveryDays = 7;

  /**
   * Whether each learner is paired with a shadow: an agent that starts from the same knowledge,
   * never learns and never walks, and plans every leg the learner plans, with the same route-choice
   * model, from the same two places. Pairs compare route length on identical origin-destination
   * pairs.
   */
  public static boolean shadowPairs = false;

  /**
   * The shares of the community's known network that learners start with, comma-separated and
   * assigned to learners in turn by agent ID. The streets nearest the agent's anchors are kept
   * first. A share below 1 also takes the city centre out of what the agent knows. "1.0" is the
   * whole community network.
   */
  public static String knowledgeLevels = "1.0";

  /**
   * The persona every learner has, by name (WORKER, STUDENT, RETIREE, FLEX); empty draws it from
   * the census as the activity tier does.
   */
  public static String fixedPersona = "";

  /** The share of the community's network the learner with this agent ID starts with. */
  public static double knowledgeShare(int agentID) {
    String[] parts = knowledgeLevels.split(",");
    return Math.max(0.0, Math.min(1.0, Double.parseDouble(parts[agentID % parts.length].trim())));
  }
}
