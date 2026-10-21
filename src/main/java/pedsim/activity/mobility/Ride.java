package pedsim.activity.mobility;

import ec.util.MersenneTwisterFast;
import pedsim.activity.parameters.ActivityPars;
import pedsim.core.parameters.Pars;
import pedsim.core.utilities.StringEnum.AgentStatus;
import sim.graph.NodeGraph;

/**
 * One ride by a mode the model does not simulate: from where it starts to where it ends, then on
 * foot to {@code destination}, after which the agent resumes as {@code resumeAs}. Where the ride
 * does not start at the agent's position, the agent first walks to it - to a stop, or back to its
 * car.
 *
 * @param from the stop boarded at, or where the car is
 * @param to the stop alighted at, or where the car is left
 * @param destination where the agent walks to from {@code to}
 * @param resumeAs the status the agent resumes once off the vehicle
 * @param byCar whether the ride is by car rather than public transport
 */
public record Ride(
    NodeGraph from, NodeGraph to, NodeGraph destination, AgentStatus resumeAs, boolean byCar) {

  /** Trip-record role of the walk to a stop, and of the walk from one. */
  public static final String TO_STOP_LEG = "to_stop";

  public static final String FROM_STOP_LEG = "from_stop";

  /** Trip-record role of the walk back to a parked car, and of the walk from one. */
  public static final String TO_CAR_LEG = "to_car";

  public static final String FROM_CAR_LEG = "from_car";

  /**
   * By public transport from {@code origin} to {@code destination}: a stop near each ({@link
   * TransitStops#near}). Null when either has none, or one stop would serve both ends, or the stop
   * near the origin is the destination itself.
   */
  public static Ride byTransit(
      NodeGraph origin, NodeGraph destination, AgentStatus resumeAs, MersenneTwisterFast random) {
    NodeGraph board = TransitStops.near(origin, random);
    NodeGraph alight = TransitStops.near(destination, random);
    if (board == null || alight == null || board.equals(alight) || board.equals(destination)) {
      return null;
    }
    return new Ride(board, alight, destination, resumeAs, false);
  }

  /**
   * By car from {@code origin}, where the car is, to {@code park}, then on foot to {@code
   * destination}. Null when there is no park, or it is the origin itself.
   */
  public static Ride byCar(
      NodeGraph origin, NodeGraph park, NodeGraph destination, AgentStatus resumeAs) {
    if (park == null || park.equals(origin)) {
      return null;
    }
    return new Ride(origin, park, destination, resumeAs, true);
  }

  /**
   * Minutes away. By public transport, waiting plus riding: half the boarding stop's headway, from
   * its weekday departures over {@link ActivityPars#transitServiceHours}, between one and fifteen
   * minutes; then the distance between the stops, scaled by the network's circuity, at {@link
   * ActivityPars#transitSpeedKmh}. By car, the same distance at {@link ActivityPars#carSpeedKmh}.
   */
  public double minutes() {
    double metres = from.getCoordinate().distance(to.getCoordinate()) * Pars.networkCircuityFactor;
    if (byCar) {
      return metres / (ActivityPars.carSpeedKmh * 1000.0 / 60.0);
    }
    int departures = Math.max(1, TransitStops.departuresAt(from));
    double wait = 0.5 * ActivityPars.transitServiceHours * 60.0 / departures;
    wait = Math.min(15.0, Math.max(1.0, wait));
    return wait + metres / (ActivityPars.transitSpeedKmh * 1000.0 / 60.0);
  }

  /** Role of the walk to where the ride starts. */
  public String walkToRole() {
    return byCar ? TO_CAR_LEG : TO_STOP_LEG;
  }

  /** Role of the walk from where the ride ends. */
  public String walkFromRole() {
    return byCar ? FROM_CAR_LEG : FROM_STOP_LEG;
  }
}
