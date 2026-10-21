package pedsim.activity.mobility;

/** How an agent makes its commute, settled once for the person by {@link ModeChoice#commute}. */
public enum CommuteMode {
  /** Walked from home to the workplace. */
  WALK,
  /** By public transport, walking to and from the stops. */
  TRANSIT,
  /** Driven to a workplace inside the limited-traffic zone, walking from the car park. */
  CAR_TO_ZONE,
  /** By any other mode, putting no one on the street. */
  OFF_STREET;

  /** Whether the commute puts the agent on the street at all. */
  public boolean onStreet() {
    return this != OFF_STREET;
  }
}
