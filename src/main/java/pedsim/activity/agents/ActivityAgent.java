package pedsim.activity.agents;

import java.time.LocalDateTime;
import java.util.EnumMap;
import java.util.LinkedHashMap;
import java.util.Map;
import pedsim.activity.engine.PedSimCityActivity;
import pedsim.activity.parameters.ActivityPars;
import pedsim.core.agents.Agent;
import pedsim.core.engine.PedSimCity;
import pedsim.core.parameters.TimePars;
import pedsim.core.utilities.StringEnum;
import pedsim.core.utilities.StringEnum.AgentStatus;
import pedsim.transit.TransitStop;
import sim.engine.SimState;
import sim.graph.NodeGraph;

/**
 * Pedestrian agent for the activity-based model. Follows a 24h activity pattern organised as
 * home-based tours:
 *
 * <ul>
 *   <li><b>Persona</b> (worker / student / retiree / flex) decides whether, when and for how long
 *       the agent attends a mandatory activity, and its walking speed;
 *   <li><b>Daily agenda</b>: on release the agent plans a small tour of discretionary activities
 *       (purpose-typed); after each stay it chains directly to the next stop instead of returning
 *       home first, and goes home when the agenda is exhausted;
 *   <li><b>Purpose-typed destinations</b>: discretionary destinations are weighted by the
 *       OSM-tag-derived per-node attraction of the current purpose (see {@code PoiClassifier}),
 *       degrading to uniform selection when no tags are available — the census only shapes the
 *       population, never destinations;
 *   <li><b>Habitual places</b>: a small set of favourite destinations per purpose is revisited,
 *       with a probability that grows as the agent's map fills and a preference for the places it
 *       already visits most, so agents develop routine geographies;
 *   <li><b>Multi-modal transit</b>: long-distance city trips (&gt; 800m) may switch to METRO, TRAM
 *       or BUS routing when the transit network is active.
 * </ul>
 *
 * <p>No vulnerability or lighting behaviour — those belong to the night module.
 */
public class ActivityAgent extends Agent {

  private static final long serialVersionUID = 1L;

  // Multi-modal transit tracking fields
  public NodeGraph ultimateDestinationNode;
  public TransitStop boardingStop;
  public TransitStop egressStop;

  /** Socio-demographic persona; {@code null} for agents created outside {@code ActivityPopulate}. */
  protected Persona persona;

  /** Individual walking-speed multiplier: persona base × personal noise. */
  private double speedFactor = 1.0;

  /** Purpose of the current discretionary trip; {@code null} on work and home legs. */
  protected ActivityPurpose currentPurpose;

  /** Remaining discretionary stops of the current tour; {@code null} until first release. */
  protected DailyAgenda agenda;

  /**
   * Habitually revisited destinations per purpose, each with the number of visits it has had.
   * Insertion-ordered so that iteration, and therefore the weighted draw over it, is reproducible.
   */
  private final Map<ActivityPurpose, LinkedHashMap<NodeGraph, Integer>> favouritePlaces =
      new EnumMap<>(ActivityPurpose.class);

  public ActivityAgent(PedSimCity state, boolean registerSpatial) {
    super(state, registerSpatial);
  }

  @Override
  public void step(SimState state) {
    if (isWaiting()) {
      return;
    }

    // Check if agent arrived at the transit boarding platform (Leg 1 completed)
    if (boardingStop != null && status == AgentStatus.WALKING_ALONE) {
      boolean atPlatform = reachedDestination.get() 
          || (destinationNode != null && destinationNode.getID() == boardingStop.snappedNodeId)
          || (currentLocation != null && currentLocation.getGeometry().getCoordinate().distance(boardingStop.snappedNodeGraph.getCoordinate()) < 20.0);

      if (atPlatform) {
        reachedDestination.set(false);
        destinationNode = ultimateDestinationNode;
        TransitStop platformStop = boardingStop;
        boardingStop = null;
        
        setStatus(AgentStatus.WAITING);
        platformStop.waitingPassengers.add(this);
        return;
      }
    }

    super.step(state);
  }

  @Override
  protected synchronized void planTrip() {
    super.planTrip();

    if (reachedDestination.get() || destinationNode == null || originNode == null) {
      return;
    }

    // Evaluate multi-modal transit for trips longer than ~800 meters when transit network is active
    if (pedsim.core.parameters.RouteChoicePars.usePublicTransport && !PedSimCityActivity.allTransitStops.isEmpty() && boardingStop == null && egressStop == null) {
      double tripDist = originNode.getCoordinate().distance(destinationNode.getCoordinate());
      if (tripDist > 800.0) {
        TransitStop bStop = findNearestStop(originNode, 600.0);
        TransitStop eStop = findNearestStop(destinationNode, 600.0);

        if (bStop != null && eStop != null && bStop != eStop 
            && bStop.snappedNodeGraph != null && eStop.snappedNodeGraph != null) {
          
          double transitDist = bStop.snappedNodeGraph.getCoordinate().distance(eStop.snappedNodeGraph.getCoordinate());
          if (transitDist > 400.0) {
            String sharedMode = null;
            double splitProbability = 0.0;

            if (bStop.servesMode("METRO") && eStop.servesMode("METRO")) {
              sharedMode = "METRO";
              splitProbability = 0.45; // 45% Metro capture rate along M1 corridor
            } else if (bStop.servesMode("TRAM") && eStop.servesMode("TRAM")) {
              sharedMode = "TRAM";
              splitProbability = 0.35; // 35% Tram capture rate along tram lines
            } else if (bStop.servesMode("BUS") && eStop.servesMode("BUS")) {
              sharedMode = "BUS";
              splitProbability = 0.30; // 30% Bus capture rate across urban bus network
            }

            if (sharedMode != null && state.random.nextDouble() < splitProbability) {
              // Agent chooses transit -> Execute Leg 1 (Walk to boarding station)
              ultimateDestinationNode = destinationNode;
              boardingStop = bStop;
              egressStop = eStop;
              destinationNode = bStop.snappedNodeGraph;
              PedSimCityActivity.agentTransitDestinations.put(this, eStop);
              if (originNode != null && destinationNode != null && originNode.getID() == destinationNode.getID()) {
                reachedDestination.set(true);
              } else {
                reinitializeMovementPath();
              }
              return;

            }
          }
        }
      }
    }

    PedSimCityActivity.countTrip("WALK");
  }


  private TransitStop findNearestStop(NodeGraph node, double maxRadius) {
    TransitStop bestStop = null;
    double bestDist = maxRadius;
    for (TransitStop stop : PedSimCityActivity.allTransitStops) {
      if (stop.snappedNodeGraph != null) {
        double d = node.getCoordinate().distance(stop.snappedNodeGraph.getCoordinate());
        if (d < bestDist) {
          bestDist = d;
          bestStop = stop;
        }
      }
    }
    return bestStop;
  }

  // ----------------------------------------------------------------
  // Persona
  // ----------------------------------------------------------------

  /** Assigns the persona and derives the individual walking speed (±10% personal noise). */
  public void setPersona(Persona persona) {
    this.persona = persona;
    this.speedFactor = persona.getSpeedFactor() * (0.90 + 0.20 * random.nextDouble());
  }

  public Persona getPersona() {
    return persona;
  }

  @Override
  public double getSpeedFactor() {
    return speedFactor;
  }

  /** Reads the activity 24h clock so destination selection and work-targeting follow time of day. */
  @Override
  protected boolean isDark() {
    return state instanceof PedSimCityActivity activityState && activityState.isDark;
  }

  /**
   * Work (or study) trip only when the persona attends today (weekday) and the current time falls
   * inside its start window — an agent released at 5 PM no longer commutes. Agents without a
   * persona keep the core rule.
   */
  @Override
  protected boolean shouldGoToWork() {
    if (workNode == null || hasWorkedToday || isDark()) {
      return false;
    }
    if (persona == null) {
      return super.shouldGoToWork();
    }
    LocalDateTime now = now();
    return persona.worksOn(now.getDayOfWeek())
        && persona.isWithinMandatoryStartWindow(hourOf(now));
  }

  // ----------------------------------------------------------------
  // Tour: agenda building and trip chaining
  // ----------------------------------------------------------------

  /**
   * How many legs this agent's tour is expected to walk if released now.
   *
   * <p>Read by the release manager so the metres budget is charged for the tour rather than for its
   * first leg. It asks the same questions {@link #startWalkingAlone} will ask a moment later, that
   * is whether the commute happens and what the agenda will hold, but answers them in expectation,
   * since the agenda does not exist yet at release time.
   *
   * @return the expected number of walked legs, at least two
   */
  public double expectedTourLegs() {
    boolean rainy = state instanceof PedSimCityActivity activityState && activityState.isRainyNow();
    return DailyAgenda.expectedLegs(persona, shouldGoToWork(), rainy);
  }

  /** Builds the tour agenda when the release manager sends this agent out. */
  @Override
  public void startWalkingAlone() {
    if (persona != null) {
      boolean rainy =
          state instanceof PedSimCityActivity activityState && activityState.isRainyNow();
      agenda = DailyAgenda.build(persona, hourOf(now()), shouldGoToWork(), random, rainy);
    }
    currentPurpose = null;
    super.startWalkingAlone();
  }

  /**
   * Called when the stay at the current destination ends. Instead of always returning home, the
   * agent chains to the next open activity on its agenda (trip starts from the current location);
   * home only when the agenda is exhausted or the next leg cannot be planned.
   */
  @Override
  protected void goHome() {
    ActivityPurpose next =
        (agenda != null) ? agenda.pollOpenActivity(hourOf(now())) : null;
    if (next == null || lastDestination == null) {
      currentPurpose = null;
      super.goHome();
      return;
    }

    currentPurpose = next;
    if (!startChainedTrip(null)) {
      currentPurpose = null;
      super.goHome();
    }
  }

  /**
   * Starts a chained leg from the current location (mirrors core {@code defineOrigin} for
   * GOING_HOME). A {@code null} destination samples a discretionary one for the current purpose.
   * Returns {@code false} — leaving status untouched — when no destination could be planned.
   */
  private boolean startChainedTrip(NodeGraph fixedDestination) {
    if (lastDestination == null) {
      return false;
    }
    originNode = lastDestination;
    if (currentLocation.getGeometry().getCoordinate() != lastDestination.getCoordinate()) {
      currentLocation.geometry = lastDestination.getMasonGeometry().geometry;
    }

    destinationNode = fixedDestination;
    if (destinationNode == null) {
      defineRandomDestination();
    }
    if (destinationNode == null) {
      return false;
    }

    state.agentsWalking.add(this);
    status = AgentStatus.WALKING_ALONE;

    if (destinationNode.getID() == originNode.getID()) {
      reachedDestination.set(true);
      return true;
    }

    planRoute();
    spookLocations.clear();
    tripStartStep = state.schedule.getSteps();
    agentMovement = createMovement();
    agentMovement.initialisePath(getRoute());
    return true;
  }

  /** The tour is over: drop the agenda so the next release starts fresh. */
  @Override
  protected void handleReachedHome() {
    agenda = null;
    currentPurpose = null;
    super.handleReachedHome();
  }

  /**
   * Stay durations by activity: the persona's mandatory-stay range at work/study, the purpose's
   * lognormal stay for discretionary stops, the core uniform draw otherwise.
   */
  @Override
  protected void calculateTimeAtDestination(long steps) {
    if (persona != null && lastDestination != null && lastDestination.equals(workNode)) {
      timeAtDestination =
          persona.sampleMandatoryStayMinutes(random) * TimePars.MINUTE_TO_STEPS + steps;
    } else if (currentPurpose != null) {
      timeAtDestination =
          currentPurpose.sampleStayMinutes(random) * TimePars.MINUTE_TO_STEPS + steps;
    } else {
      super.calculateTimeAtDestination(steps);
    }
  }

  // ----------------------------------------------------------------
  // Destination choice: purpose weights and habitual places
  // ----------------------------------------------------------------

  /**
   * Discretionary destination choice: resolves the trip's purpose (first agenda entry when this is
   * the tour's first leg), then either returns to a place the agent already knows or samples a new
   * one at the released distance, and records the visit either way.
   *
   * <p>The purpose does not scale that distance. It used to; see the note in
   * {@link ActivityPurpose} for why it no longer does.
   */
  @Override
  protected void defineRandomDestination() {
    ensureCurrentPurpose();
    if (tryHabitualDestination()) {
      return;
    }
    super.defineRandomDestination();
    rememberFavourite(currentPurpose, destinationNode);
  }

  /**
   * Resolves the purpose of the tour's first discretionary leg from the agenda. Chained legs get
   * their purpose in {@link #goHome()}; agents without persona/agenda keep {@code null} (uniform
   * destination weighting).
   */
  protected void ensureCurrentPurpose() {
    if (currentPurpose == null && agenda != null && persona != null) {
      currentPurpose = agenda.pollOpenActivity(hourOf(now()));
      if (currentPurpose == null) {
        // Released with an empty (or already-closed) agenda: take a walk.
        currentPurpose = ActivityPurpose.STROLL;
      }
    }
  }

  private boolean tryHabitualDestination() {
    if (currentPurpose == null || currentPurpose == ActivityPurpose.STROLL) {
      return false;
    }
    if (random.nextDouble() >= returnProbability()) {
      return false;
    }
    Map<NodeGraph, Integer> favourites = favouritePlaces.get(currentPurpose);
    if (favourites == null || favourites.isEmpty()) {
      return false;
    }
    NodeGraph pick = preferentialReturn(favourites);
    if (pick == null || (originNode != null && pick.getID() == originNode.getID())) {
      return false;
    }
    destinationNode = pick;
    favourites.merge(pick, 1, Integer::sum);
    return true;
  }

  /**
   * Probability that this trip goes somewhere the agent already knows rather than to a new place.
   *
   * <p>Not a constant: Song et al. (2010) measure the complementary exploration probability on
   * mobile-phone trajectories as {@code P_new = rho * S^-gamma}, where {@code S} is the number of
   * distinct locations the individual has already visited. The exponent is a property of the
   * person, not of the activity type, so {@code S} counts every remembered place across purposes,
   * even though the return itself is confined to the current purpose's list. An agent that knows
   * nowhere explores with certainty; one holding the full 25 familiar places of
   * {@link ActivityPars#familiarLocationCapacity} returns about seven times in ten.
   */
  private double returnProbability() {
    int distinctPlaces = 0;
    for (Map<NodeGraph, Integer> places : favouritePlaces.values()) {
      distinctPlaces += places.size();
    }
    if (distinctPlaces == 0) {
      return 0.0;
    }
    return 1.0
        - ActivityPars.explorationRho * Math.pow(distinctPlaces, -ActivityPars.explorationGamma);
  }

  /**
   * Draws one of the purpose's favourite places with probability proportional to how often it has
   * been visited. Song et al. call this preferential return, and show that picking uniformly
   * instead - every known place equally likely - flattens the visitation frequencies and destroys
   * the Zipf law they follow in the data.
   */
  private NodeGraph preferentialReturn(Map<NodeGraph, Integer> favourites) {
    int totalVisits = 0;
    for (int visits : favourites.values()) {
      totalVisits += visits;
    }
    if (totalVisits <= 0) {
      return null;
    }
    int draw = random.nextInt(totalVisits);
    int cumulative = 0;
    for (Map.Entry<NodeGraph, Integer> favourite : favourites.entrySet()) {
      cumulative += favourite.getValue();
      if (draw < cumulative) {
        return favourite.getKey();
      }
    }
    return null;
  }

  /**
   * Records a visit, admitting the place to the agent's familiar set if it is new.
   *
   * <p>The set has a fixed size across all purposes (see
   * {@link ActivityPars#familiarLocationCapacity}) and turns over rather than filling up: when it
   * is full, the least-visited place anywhere in it makes room for the new one. Without the
   * turnover an agent stops learning the moment it is full, which is both wrong and self-
   * reinforcing, since {@link #returnProbability()} reads the size of this set.
   */
  private void rememberFavourite(ActivityPurpose purpose, NodeGraph node) {
    if (purpose == null || purpose == ActivityPurpose.STROLL || node == null) {
      return;
    }
    Map<NodeGraph, Integer> favourites =
        favouritePlaces.computeIfAbsent(purpose, p -> new LinkedHashMap<>());
    if (favourites.containsKey(node)) {
      favourites.merge(node, 1, Integer::sum);
      return;
    }
    if (countFamiliarPlaces() >= ActivityPars.familiarLocationCapacity && !evictLeastVisited(node)) {
      return;
    }
    favourites.put(node, 1);
  }

  /** How many distinct places the agent currently holds, across every purpose. */
  private int countFamiliarPlaces() {
    int total = 0;
    for (Map<NodeGraph, Integer> places : favouritePlaces.values()) {
      total += places.size();
    }
    return total;
  }

  /**
   * Drops the least-visited familiar place to make room for {@code incoming}. Ties go to the one
   * learned earliest, which the insertion-ordered maps make deterministic.
   *
   * @return whether a place was actually dropped
   */
  private boolean evictLeastVisited(NodeGraph incoming) {
    Map<NodeGraph, Integer> leastVisitedIn = null;
    NodeGraph leastVisited = null;
    int fewestVisits = Integer.MAX_VALUE;
    for (Map<NodeGraph, Integer> places : favouritePlaces.values()) {
      for (Map.Entry<NodeGraph, Integer> place : places.entrySet()) {
        if (!place.getKey().equals(incoming) && place.getValue() < fewestVisits) {
          fewestVisits = place.getValue();
          leastVisited = place.getKey();
          leastVisitedIn = places;
        }
      }
    }
    if (leastVisited == null) {
      return false;
    }
    leastVisitedIn.remove(leastVisited);
    return true;
  }

  /**
   * Weights candidate destinations by the current purpose's OSM-tag-derived per-node attraction.
   * STROLL is deliberately uniform. Returns 0.0 (uniform selection) when no purpose is set or the
   * city carries no use tags — the census plays no role in destination choice; it only shapes the
   * population (home spawning, headcount, vulnerability).
   */
  @Override
  protected double getPOIWeight(NodeGraph node) {
    if (currentPurpose == null || currentPurpose == ActivityPurpose.STROLL) {
      return 0.0;
    }
    Map<NodeGraph, Double> purposeWeights =
        PedSimCityActivity.nodesPurposeWeight.get(currentPurpose);
    if (purposeWeights == null || purposeWeights.isEmpty()) {
      return 0.0;
    }
    return purposeWeights.getOrDefault(node, 0.0);
  }

  // ----------------------------------------------------------------
  // Reporting
  // ----------------------------------------------------------------

  /**
   * Activity volumes are tallied per hour of day (h01–h24); the agent type stays {@code DEFAULT}
   * (no vulnerable / learner split). The hour is read when a trip is recorded, so a trip counts
   * towards the hour it completes in.
   */
  @Override
  public Enum<?> getSimulationScenario() {
    return StringEnum.Hour.of(TimePars.getTime(state.schedule.getSteps()).toLocalTime().getHour());
  }

  // ----------------------------------------------------------------
  // Helpers
  // ----------------------------------------------------------------

  private LocalDateTime now() {
    return TimePars.getTime(state.schedule.getSteps());
  }

  private static double hourOf(LocalDateTime time) {
    return time.getHour() + time.getMinute() / 60.0;
  }
}
