package pedsim.transit;

import java.io.Serializable;
import java.util.ArrayList;
import java.util.List;
import pedsim.core.agents.Agent;
import sim.graph.NodeGraph;

/**
 * Represents a physical public transit station or stop snapped to a pedestrian street node.
 * Maintains a platform waiting queue of agents who are waiting to board a transit vehicle.
 */
public class TransitStop implements Serializable {

  private static final long serialVersionUID = 1L;

  public final String stopId;
  public final String stopName;
  public final int snappedNodeId;
  public final double x;
  public final double y;
  public final String modesServed;
  public final String routesServed;

  // The actual pedestrian graph node representing this stop on the street network
  public NodeGraph snappedNodeGraph;

  // Queue of pedestrian agents currently standing on the platform waiting for a vehicle
  public final List<Agent> waitingPassengers = new ArrayList<>();

  public TransitStop(
      String stopId,
      String stopName,
      int snappedNodeId,
      double x,
      double y,
      String modesServed,
      String routesServed) {
    this.stopId = stopId;
    this.stopName = stopName;
    this.snappedNodeId = snappedNodeId;
    this.x = x;
    this.y = y;
    this.modesServed = modesServed;
    this.routesServed = routesServed;
  }

  /**
   * Checks if this stop serves a particular transit mode ("METRO", "TRAM", "BUS").
   */
  public boolean servesMode(String mode) {
    if (modesServed == null) return false;
    return modesServed.toUpperCase().contains(mode.toUpperCase());
  }

  /**
   * Checks if this stop serves a particular transit line / route (e.g., "M1", "4", "16CD").
   */
  public boolean servesRoute(String route) {
    if (routesServed == null) return false;
    return routesServed.toUpperCase().contains(route.toUpperCase());
  }

  @Override
  public String toString() {
    return String.format(
        "TransitStop[%s: %s (Node %d) Modes=%s]", stopId, stopName, snappedNodeId, modesServed);
  }
}
