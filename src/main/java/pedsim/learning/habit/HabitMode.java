package pedsim.learning.habit;

/** What, besides the known network, makes a learner prefer some streets over others. */
public enum HabitMode {
  /** Nothing: the learner prices every street at its length. */
  NONE,
  /** Streets it remembers vividly cost less. */
  REMEMBERED,
  /** Streets on routes that turned out better than the first one it walked cost less. */
  REWARDED
}
