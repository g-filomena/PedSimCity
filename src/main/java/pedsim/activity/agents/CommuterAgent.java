package pedsim.activity.agents;

import pedsim.core.agents.Agent;
import pedsim.core.agents.OdAgent;
import pedsim.core.engine.PedSimCity;
import pedsim.core.parameters.TimePars;

/**
 * An agent with a job and a day shaped around it: out to the workplace once, a long stay, home.
 *
 * <p>All of this lived on {@link Agent} — the worked-today latch, the rule for when to set off for
 * work, the hours spent there — which made employment a property of every pedestrian the model can
 * represent. It is not: {@link OdAgent} walks a fixed list of origin-destination pairs and has no
 * day at all, and this module's retirees and flex adults have no workplace, which is why
 * {@link Agent#cognitiveAnchors()} had to stop assuming one.
 *
 * <p>It lives here, and not in {@code core}, because a working day is an <i>activity
 * programme</i> — the smallest one the model has. Core is the skeleton every module builds on and is not meant to
 * produce a day by itself: its default agent is a bare {@link Agent}, which goes somewhere it knows
 * and comes back. Giving core a commuter made employment the default property of a pedestrian in
 * the one layer that has no data to justify it.
 *
 * <p>What stayed on {@code Agent} is the workplace as a <i>place</i> — {@code workNode}, and its
 * part in the cognitive map's anchors. A location someone knows is not the same claim as a working
 * day, and the cognitive map wants the first without the second.
 *
 * <p>The stay lengths here have no source. Six to nine hours is a working day as anyone would
 * guess it, not as anything measured; {@link ActivityAgent} overrides it with the persona's
 * mandatory-stay distribution, which at least comes from somewhere.
 */
public class CommuterAgent extends Agent {

  private static final long serialVersionUID = 1L;

  /** Whether the commute has already been made today. Cleared when the agent gets home. */
  protected boolean hasWorkedToday = false;

  public CommuterAgent(PedSimCity state) {
    super(state);
  }

  public CommuterAgent(PedSimCity state, boolean registerSpatial) {
    super(state, registerSpatial);
  }

  /**
   * Whether the next non-home trip should target the work node: a work node exists and the agent
   * has not worked today. Activity-based modules refine this with persona work-start windows and
   * day-of-week.
   *
   * <p>Darkness plays no part: people walk to and from work in the dark for months of the year.
   */
  protected boolean shouldGoToWork() {
    return workNode != null && !hasWorkedToday;
  }

  /** The commute comes first; anything else is a trip to somewhere the agent knows. */
  @Override
  protected void defineOutboundDestination() {
    if (shouldGoToWork()) {
      destinationNode = workNode;
    } else {
      super.defineOutboundDestination();
    }
  }

  /** Arriving at the workplace is what spends the day's commute. */
  @Override
  protected void handleReachedSoloDestination() {
    if (lastDestination != null && lastDestination.equals(workNode)) {
      hasWorkedToday = true;
    }
    super.handleReachedSoloDestination();
  }

  @Override
  protected void handleReachedHome() {
    hasWorkedToday = false; // Reset for the next day
    super.handleReachedHome();
  }

  /** Six to nine hours at the workplace; elsewhere, the core stay. */
  @Override
  protected void calculateTimeAtDestination(long steps) {
    if (lastDestination != null && lastDestination.equals(workNode)) {
      int randomMinutes = 360 + random.nextInt(181);
      timeAtDestination = (randomMinutes * TimePars.MINUTE_TO_STEPS) + steps;
      return;
    }
    super.calculateTimeAtDestination(steps);
  }
}
