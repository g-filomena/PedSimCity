package pedsim.activity.mobility;

import ec.util.MersenneTwisterFast;
import pedsim.activity.parameters.ActivityPars;
import sim.graph.NodeGraph;

/**
 * Layer 4 for what is not walked: which other mode a commute or a leg goes by, when that mode puts
 * someone on the street. Whether a leg is walked at all is the walk-share curves', asked by the
 * agent; what is decided here is only what follows a no.
 *
 * <p>Every share is a city fact, zero unless the city file states it ({@link ActivityPars}).
 */
public final class ModeChoice {

  private ModeChoice() {}

  /**
   * How a commute is made, given whether it is walked. A commuter not walking goes by public
   * transport when stops serve home and workplace, against the city's transit share; otherwise
   * drives when the workplace is inside the limited-traffic zone, against the car share; otherwise
   * by a mode that puts no one on the street. Someone who arrives from outside the city, at a
   * station, stop or car park, rides what it does not walk where it can and walks the rest.
   *
   * @param walks whether the commute is walked, already drawn against the commute curve
   * @param home where the commute starts
   * @param work the workplace or place of study
   * @param student whether it is a study commute
   * @param inbound whether the agent lives outside the city
   * @param random the agent's generator
   * @return the commute mode
   */
  public static CommuteMode commute(
      boolean walks,
      NodeGraph home,
      NodeGraph work,
      boolean student,
      boolean inbound,
      MersenneTwisterFast random) {
    if (inbound) {
      return !walks && TransitStops.servesBoth(home, work) ? CommuteMode.TRANSIT : CommuteMode.WALK;
    }
    if (walks) {
      return CommuteMode.WALK;
    }
    if (TransitStops.servesBoth(home, work)
        && random.nextDouble()
            < (student ? ActivityPars.transitShareStudent : ActivityPars.transitShareWorker)) {
      return CommuteMode.TRANSIT;
    }
    if (CarParks.available()
        && CarParks.inZone(work)
        && random.nextDouble()
            < (student ? ActivityPars.carShareStudent : ActivityPars.carShareWorker)) {
      return CommuteMode.CAR_TO_ZONE;
    }
    return CommuteMode.OFF_STREET;
  }

  /** A discretionary leg not walked goes by public transport, when stops serve both ends. */
  public static boolean takesTransit(
      NodeGraph origin, NodeGraph destination, MersenneTwisterFast random) {
    return ActivityPars.transitShareDiscretionary > 0.0
        && TransitStops.servesBoth(origin, destination)
        && random.nextDouble() < ActivityPars.transitShareDiscretionary;
  }

  /**
   * A discretionary leg neither walked nor by public transport is driven, and walked from a car park,
   * when its destination is inside the limited-traffic zone.
   */
  public static boolean drivesToZone(NodeGraph destination, MersenneTwisterFast random) {
    return ActivityPars.carShareDiscretionary > 0.0
        && CarParks.available()
        && CarParks.inZone(destination)
        && random.nextDouble() < ActivityPars.carShareDiscretionary;
  }
}
