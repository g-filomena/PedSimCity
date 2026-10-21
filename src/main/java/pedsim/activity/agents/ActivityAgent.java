package pedsim.activity.agents;

import java.time.LocalDateTime;
import java.util.ArrayList;
import java.util.EnumMap;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import pedsim.activity.engine.InboundCommuters;
import pedsim.activity.engine.PedSimCityActivity;
import pedsim.activity.mobility.CarParks;
import pedsim.activity.mobility.CommuteMode;
import pedsim.activity.mobility.ModeChoice;
import pedsim.activity.mobility.Ride;
import pedsim.activity.mobility.TransitStops;
import pedsim.activity.parameters.ActivityPars;
import pedsim.core.agents.Agent;
import pedsim.core.cognition.cognitivemap.SharedCognitiveMap;
import pedsim.core.engine.PedSimCity;
import pedsim.core.parameters.Pars;
import pedsim.core.parameters.TimePars;
import pedsim.core.utilities.StringEnum;
import pedsim.core.utilities.StringEnum.AgentStatus;
import sim.graph.Graph;
import sim.graph.NodeGraph;
import sim.graph.NodesLookup;
import sim.routing.Route;

/**
 * Pedestrian agent for the activity-based model. Follows a 24h activity pattern organised as
 * home-based trip chains:
 *
 * <ul>
 *   <li><b>Persona</b> (worker / student / retiree / flex) decides whether, when and for how long
 *       the agent attends a mandatory activity, and its walking speed;
 *   <li><b>Daily agenda</b>: on release the agent plans a small trip chain of discretionary activities
 *       (purpose-typed); after each stay it chains directly to the next stop instead of returning
 *       home first, and goes home when the agenda is exhausted;
 *   <li><b>Purpose-typed destinations</b>: discretionary destinations are weighted by the
 *       OSM-tag-derived per-node attraction of the current purpose (see {@code PoiClassifier}),
 *       degrading to uniform selection when no tags are available — the census only shapes the
 *       population, never destinations;
 *   <li><b>Habitual places</b>: a small set of favourite destinations per purpose is revisited,
 *       with a probability that grows as the agent's map fills and a preference for the places it
 *       already visits most, so agents develop routine geographies;
 * </ul>
 *
 * <p>The working day is part of it: an agent with a workplace goes there once a day, stays for its
 * persona's mandatory-stay range, and comes home. That lives here rather than in {@code core}
 * because a working day is an activity programme - the smallest one the model has - and core's
 * default agent is a bare {@link Agent} that goes somewhere it knows and comes back. What stays on
 * {@code Agent} is the workplace as a <i>place</i>: {@code workNode} and its part in the cognitive
 * map's anchors, which is a location someone knows rather than a job.
 *
 * <p>No vulnerability or lighting behaviour — those belong to the night module.
 */
public class ActivityAgent extends Agent {

  private static final long serialVersionUID = 1L;

  /** Whether the commute has already been made today. Cleared when the agent gets home. */
  protected boolean hasWorkedToday = false;

  /** Socio-demographic persona; {@code null} for agents created outside {@code ActivityPopulate}. */
  protected Persona persona;

  /** Individual walking-speed multiplier: persona base × personal noise. */
  private double speedFactor = 1.0;

  /** Purpose of the current discretionary trip; {@code null} on work and home legs. */
  protected ActivityPurpose currentPurpose;

  /** Remaining discretionary stops of the current trip chain; {@code null} until first release. */
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

  // ----------------------------------------------------------------
  // Persona
  // ----------------------------------------------------------------

  /**
   * Whether this agent is one of the people who walk their commute.
   *
   * <p>Decided once, when home and workplace are both known, and fixed thereafter: someone either
   * walks their commute or does not, and re-rolling it each morning would make it a coin flip
   * rather than a property of that person and that journey.
   *
   * <p>It is decided by <em>how far the commute is</em>, through the walk-share curve. Distance does
   * not decide whether a journey happens, it decides how the journey is made: a five-kilometre
   * commute is not impossible, it is simply not walked.
   *
   * <p>The population share this produces is a <em>prediction to check</em> rather than a rate
   * imposed on the agents. {@code ActivityTravelDemand.prepare} logs it against the ISTAT figures
   * once per simulated day; a poor match is a finding about the curve, not a number to adjust.
   *
   * <p>A commuter who does not walk goes by public transport, walking to and from the stops; drives
   * to a workplace inside the limited-traffic zone and walks from the car park; or goes by some
   * other mode and is then absent from the street ({@link ModeChoice#commute}).
   */
  protected CommuteMode commuteMode = CommuteMode.WALK;

  /** The ride under way, by public transport or by car; set by startRide, cleared on alighting. */
  private Ride ride;

  /** The structure holding this agent's car on a working day, when it drives into the zone. */
  private NodeGraph commuteCarPark;

  /** Gives this commuter's car a space ({@link CarParks#reserve}). */
  public void assignCommuteCarPark(NodeGraph park) {
    this.commuteCarPark = park;
  }

  /**
   * No space was left for this commuter's car in the zone, so it does not drive there: it takes
   * public transport where stops serve home and workplace, and otherwise is off the street.
   */
  public void loseCommuteCarPark() {
    commuteCarPark = null;
    commuteMode =
        TransitStops.servesBoth(homeNode, workNode) ? CommuteMode.TRANSIT : CommuteMode.OFF_STREET;
  }

  /** Whether a leg of the current outing went by public transport; cleared on reaching home. */
  private boolean outByTransit = false;

  /** Whether the current outing went by car, and the structure the car was left in. */
  private boolean outByCar = false;

  private NodeGraph parkedAt;

  // The lunch break from work: when it starts, when the working day then ends, whether the agent
  // is on it now, and whether it is eaten at home and for how long.
  private boolean lunchPending = false;
  private boolean onLunchBreak = false;
  private boolean lunchAtHome = false;
  private int lunchHomeStayMinutes = 0;
  private double workEndsAtStep = -1;

  /**
   * The commuting flow this agent belongs to when it lives outside the city ({@link
   * InboundCommuters}); null for a resident. Its home node is then where it arrives in the city.
   */
  private InboundCommuters.Flow inboundFlow;

  /**
   * Makes this agent one of the people who come into the city every day: it arrives at its home
   * node, by the flow's mode, at a time drawn from the flow's departure and duration bands.
   */
  public void setInbound(InboundCommuters.Flow flow) {
    this.inboundFlow = flow;
  }

  /** Whether this agent lives outside the city and only commutes into it. */
  public boolean isInbound() {
    return inboundFlow != null;
  }

  /**
   * Whether this agent is staying in the city as a tourist ({@link #makeTourist}); its home node is
   * then where it sleeps.
   */
  private boolean tourist = false;

  /** Today's programme, for a tourist; drawn by {@link #planTouristDay()}. */
  private TouristDay touristDay;

  /** Sights this tourist has already been to, so it goes to see something else. */
  private final java.util.Set<NodeGraph> sightsSeen = new java.util.LinkedHashSet<>();

  /** Makes this agent a tourist, sleeping at its home node. */
  public void makeTourist() {
    this.tourist = true;
  }

  /** Whether this agent is a tourist. */
  public boolean isTourist() {
    return tourist;
  }

  /** Whether this agent lives in the city: neither an inbound commuter nor a tourist. */
  public boolean isResident() {
    return !isInbound() && !tourist;
  }

  /**
   * Draws today's programme for a tourist ({@link TouristDay}) and schedules its first outing.
   * Tourists go out every day.
   */
  public void planTouristDay() {
    touristDay = TouristDay.draw(random);
    mandatoryDepartureMinute = touristDay.leaveMinute();
  }

  // The role of the leg under way, and of the next one to start.
  private String legRole = WALK_LEG;
  private String nextLegRole = WALK_LEG;

  /**
   * Whether this agent is a woman, drawn once from the home zone's census sex ratio. A demographic
   * fact about the person, alongside the persona: what any model makes of it - who is treated as
   * vulnerable after dark, how often someone travels - is that model's judgement, made elsewhere.
   */
  private boolean female;

  /** Whether this agent is a woman. Agents are adults, so this is the adult sex ratio. */
  public boolean isFemale() {
    return female;
  }

  /** Sets the agent's sex; drawn from the home zone by {@code ActivityPopulate}. */
  public void setFemale(boolean female) {
    this.female = female;
  }

  /** Assigns the persona and derives the individual walking speed (±10% personal noise). */
  public void setPersona(Persona persona) {
    this.persona = persona;
    this.speedFactor = persona.getSpeedFactor() * (0.90 + 0.20 * random.nextDouble());
  }

  /**
   * Settles whether this agent's commute is walked, from the distance between home and workplace.
   * Call once both are assigned; agents with no workplace never commute anyway.
   */
  public void decideCommuteMode() {
    if (homeNode == null || workNode == null) {
      commuteMode = CommuteMode.OFF_STREET;
      return;
    }
    // Scaled to walked metres, like the distance term in DestinationChoice: the curve was fitted
    // to NTS0308 trip lengths, which are reported journey distances, so feeding it a straight line
    // asks it the wrong question and keeps commutes the factor of the network's circuity too
    // short - which is to say it walks far more of them than anyone walks.
    double commuteMetres =
        homeNode.getCoordinate().distance(workNode.getCoordinate()) * Pars.networkCircuityFactor;
    boolean student = persona == Persona.STUDENT;
    boolean walks =
        random.nextDouble() < state.travelDemand().commuteWalkProbability(commuteMetres, student);
    commuteMode = ModeChoice.commute(walks, homeNode, workNode, student, isInbound(), random);
  }

  /**
   * Estimated metres walked on this agent's commute: the straight line home to workplace, scaled by
   * the network's measured circuity. The same estimate {@code decideCommuteMode} asks the
   * walk-share curve about, and the only one available before the route exists.
   */
  public double commuteMetres() {
    if (homeNode == null || workNode == null) {
      return 0.0;
    }
    return homeNode.getCoordinate().distance(workNode.getCoordinate()) * Pars.networkCircuityFactor;
  }

  /** Whether this agent has a workplace or place of study at all. */
  public boolean hasWorkplace() {
    return workNode != null;
  }

  /** Whether this agent walks its commute; false when it would use some other mode. */
  public boolean walksToWork() {
    return commuteMode == CommuteMode.WALK;
  }

  /** How this agent makes its commute. */
  public CommuteMode commuteMode() {
    return commuteMode;
  }

  /** Whether the commute puts this agent on the street at all. */
  private boolean commuteOnStreet() {
    return commuteMode.onStreet();
  }

  /** Minute of the day this agent sets out for its mandatory activity; -1 when it has none today. */
  protected int mandatoryDepartureMinute = -1;

  /**
   * Draws this agent's departure time for today's mandatory activity, or establishes that it has
   * none.
   *
   * <p>Having a job means going to it: the commute is generated from what the agent is - it has a
   * workplace, it walks there, and its persona attends today - rather than drawn. The only question
   * left to chance is which discretionary trips happen on top.
   *
   * <p>The minute is drawn uniformly inside the persona's mandatory start window. That window is
   * read as a departure window rather than an arrival window, which is the convention the profile
   * it replaces already used; making it an arrival window means subtracting a travel time that is
   * not known until the route exists.
   *
   * <p>It must ask exactly what {@link #shouldGoToWork()} will ask when the agent is actually
   * released, or the day's leg budget is charged for a commute chain that then does not happen.
   * Neither consults darkness: the persona's start window decides when someone leaves, and the
   * season decides whether it is light when they do.
   *
   * @param day the day being prepared
   * @return whether this agent's mandatory trip today puts it on the street, walked or by public
   *     transport
   */
  /**
   * Whether this agent travels to its mandatory activity today, <b>by any mode</b>. The day's trip
   * budget counts trips, so a commute that is driven still spends from it; whether it is walked -
   * and so whether the agent is ever released onto the street - is {@link #planMandatoryDeparture}.
   *
   * @param day the day being prepared
   */
  public boolean commutesOn(java.time.LocalDate day) {
    return persona != null
        && workNode != null
        && persona.hasMandatoryActivity()
        && persona.worksOn(day.getDayOfWeek());
  }

  public boolean planMandatoryDeparture(java.time.LocalDate day) {
    mandatoryDepartureMinute = -1;
    if (persona == null
        || workNode == null
        || !commuteOnStreet()
        || !persona.hasMandatoryActivity()
        || !persona.worksOn(day.getDayOfWeek())) {
      return false;
    }
    if (isInbound()) {
      mandatoryDepartureMinute = InboundCommuters.arrivalMinute(inboundFlow, random);
      return true;
    }
    int from = (int) Math.round(persona.getMandatoryStartEarliest() * 60.0);
    int to = (int) Math.round(persona.getMandatoryStartLatest() * 60.0);
    mandatoryDepartureMinute = to > from ? from + random.nextInt(to - from) : from;
    return true;
  }

  /** Minute of the day this agent departs for its mandatory activity, or -1. */
  public int mandatoryDepartureMinute() {
    return mandatoryDepartureMinute;
  }

  public Persona getPersona() {
    return persona;
  }

  @Override
  public double getSpeedFactor() {
    return speedFactor;
  }

  /**
   * Counts the leg against the day's darkness counters, then installs it. Every planner crosses
   * this, chained legs included.
   */
  @Override
  public void initialiseRoute(Route route) {
    if (route != null && state instanceof PedSimCityActivity activityState) {
      if (isDark()) {
        activityState.legsInDarkness.increment();
      }
    }
    super.initialiseRoute(route);
  }

  /**
   * Whether it is dark now, from the activity tier's 24h clock.
   *
   * <p>Not a core seam: core has no clock, so a core agent has no answer to give rather than a
   * {@code false} one. Darkness starts at {@link PedSimCityActivity#isDark}, which
   * {@code ActivityEngine} recomputes each step from {@code Daylight.isDark}.
   */
  protected boolean isDark() {
    return state instanceof PedSimCityActivity activityState && activityState.isDark;
  }

  /**
   * Work (or study) trip only when the persona attends today (weekday) and the current time falls
   * inside its start window — an agent released at 5 PM no longer commutes. Agents without a
   * persona keep the core rule.
   *
   * <p>Darkness is deliberately not consulted: the persona's start window says when someone sets
   * off, and the season says whether it is light when they do. Turin's sunset is before 17:00
   * through December, so much of the winter commute happens in the dark - the most routine walking
   * there is, by the population most exposed to unlit streets.
   */
  protected boolean shouldGoToWork() {
    if (workNode == null || hasWorkedToday || !commuteOnStreet()) {
      return false;
    }
    if (persona == null) {
      return true;
    }
    LocalDateTime now = now();
    if (isInbound()) {
      // Released only by its scheduled arrival, whose time comes from the commuting data.
      return persona.worksOn(now.getDayOfWeek());
    }
    return persona.worksOn(now.getDayOfWeek()) && persona.isWithinMandatoryStartWindow(hourOf(now));
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

  /**
   * Mode choice for a discretionary leg: a draw against the general walk-share curve at the distance
   * this leg turned out to be.
   *
   * <p>The commute is exempt because its mode was already settled, once and for the person, by
   * {@link #decideCommuteMode()} against a curve fitted to commuting. Asking again here would filter
   * the same journey twice, through two different curves.
   *
   * <p>Distance enters as a cost, not as a filter on which trips exist: the destination was chosen
   * before this is asked, and what is decided is how the journey to it is made. A leg that is not
   * walked goes by public transport or leaves no one on the street ({@link #travelUnwalked()}).
   */
  @Override
  protected boolean walksLeg(NodeGraph origin, NodeGraph destination) {
    if (destination == null) {
      return true;
    }
    if (isCommute(origin, destination)) {
      return commuteMode != CommuteMode.TRANSIT && commuteMode != CommuteMode.CAR_TO_ZONE;
    }
    double metres =
        origin.getCoordinate().distance(destination.getCoordinate()) * Pars.networkCircuityFactor;
    double perceived = perceivedLegMetres(origin, destination, metres);
    boolean walks = random.nextDouble() < state.travelDemand().walkProbability(perceived);
    if (isResident() && state instanceof PedSimCityActivity activityState) {
      activityState.recordModeChoice(metres, walks, homeNode);
    }
    return walks;
  }

  /**
   * The length the decision to walk weighs: the leg's metres. A module that prices a street by more
   * than its length answers with what the leg would feel like to walk.
   *
   * @param origin where the leg starts
   * @param destination where it ends
   * @param metres the leg's estimated walked length
   * @return the length to read the walk-share curve at
   */
  protected double perceivedLegMetres(NodeGraph origin, NodeGraph destination, double metres) {
    return metres;
  }

  /**
   * The way home after going out by public transport is a mode decision of its own: from the
   * workplace it is the commute's, otherwise the walk-share curve's at the distance home. An outing
   * by car goes home by car, from where it was left. An outing made on foot is walked back.
   */
  @Override
  protected boolean walksBack(NodeGraph origin, NodeGraph home) {
    if (tourist) {
      double metres = walkedMetres(origin, home);
      return random.nextDouble()
          < state.travelDemand().walkProbability(perceivedLegMetres(origin, home, metres));
    }
    if (outByCar && parkedAt != null) {
      return false;
    }
    if (!outByTransit) {
      return true;
    }
    if (origin.equals(workNode)) {
      return commuteMode != CommuteMode.TRANSIT;
    }
    double metres =
        origin.getCoordinate().distance(home.getCoordinate()) * Pars.networkCircuityFactor;
    return random.nextDouble()
        < state.travelDemand().walkProbability(perceivedLegMetres(origin, home, metres));
  }

  /**
   * A leg not walked goes by public transport when the agent takes it - a commute by its commute
   * mode, the way home after going out by it whenever stops serve both ends, any other leg against
   * {@link ActivityPars#transitShareDiscretionary} - and otherwise leaves no one on the street. The
   * way home is always made: walked when public transport does not take the agent there, since
   * nothing else in the model can.
   */
  @Override
  protected void travelUnwalked() {
    if (tourist) {
      startTouristRide();
      return;
    }
    boolean commute = isCommute(originNode, destinationNode);
    if (isGoingHome() && outByCar && parkedAt != null) {
      startRide(new Ride(parkedAt, destinationNode, destinationNode, status, true), commute);
      return;
    }
    boolean byTransit;
    if (commute) {
      byTransit = commuteMode == CommuteMode.TRANSIT;
    } else if (isGoingHome()) {
      byTransit = outByTransit && TransitStops.servesBoth(originNode, destinationNode);
    } else {
      byTransit = ModeChoice.takesTransit(originNode, destinationNode, random);
    }
    if (byTransit) {
      if (!startTransitLeg(commute)) {
        // One stop serves both ends: the leg is short enough to walk.
        reinitializeMovementPath();
      }
      return;
    }
    if (!isGoingHome()
        && (commute
            ? commuteMode == CommuteMode.CAR_TO_ZONE
            : ModeChoice.drivesToZone(destinationNode, random))
        && startCarLeg(commute)) {
      return;
    }
    if (isGoingHome()) {
      reinitializeMovementPath();
      return;
    }
    countOffStreet();
    super.travelUnwalked();
  }

  /** A chain that never set off leaves no agenda behind. */
  @Override
  protected void abandonUnwalkedLeg() {
    agenda = null;
    currentPurpose = null;
    super.abandonUnwalkedLeg();
  }

  /**
   * A leg a tourist does not walk: by public transport where stops serve both ends, otherwise by
   * taxi, which leaves it at the door. A tourist has no car of its own.
   */
  private void startTouristRide() {
    Ride ride =
        TransitStops.servesBoth(originNode, destinationNode)
            ? Ride.byTransit(originNode, destinationNode, status, random)
            : null;
    if (ride == null) {
      ride = new Ride(originNode, destinationNode, destinationNode, status, true);
    }
    startRide(ride, false);
  }

  /**
   * Whether a leg is made as the commute is: to the workplace, or from it home, except to and from
   * a lunch eaten out, which are legs like any other.
   */
  private boolean isCommute(NodeGraph origin, NodeGraph destination) {
    if (workNode == null || (onLunchBreak && !lunchAtHome)) {
      return false;
    }
    return workNode.equals(destination)
        || (workNode.equals(origin) && (isGoingHome() || homeNode.equals(destination)));
  }

  /**
   * Drives from where the agent is to a structure near the destination, and walks from it: a
   * commute to the space it was given, any other leg to a space commuters left free. Home from the
   * workplace, it walks back to the car and drives it home.
   */
  private boolean startCarLeg(boolean commute) {
    if (outByCar && parkedAt != null && destinationNode.equals(homeNode)) {
      startRide(new Ride(parkedAt, homeNode, homeNode, status, true), countsAsCommute(commute));
      return true;
    }
    NodeGraph park = commute ? commuteCarPark : CarParks.nearFree(destinationNode, random);
    Ride car = Ride.byCar(originNode, park, destinationNode, status);
    if (car == null) {
      return false;
    }
    startRide(car, countsAsCommute(commute));
    return true;
  }

  /**
   * Sets off by public transport from {@code originNode} to {@code destinationNode} ({@link
   * Ride#byTransit}). Returns false, changing nothing, when one stop would serve both ends.
   */
  private boolean startTransitLeg(boolean commute) {
    Ride transit = Ride.byTransit(originNode, destinationNode, status, random);
    if (transit == null) {
      return false;
    }
    startRide(transit, countsAsCommute(commute));
    return true;
  }

  /** A journey home for lunch and back is made as the commute is, but is not counted as one. */
  private boolean countsAsCommute(boolean commute) {
    return commute && !onLunchBreak;
  }

  /** Takes {@code next}: walks to where it starts unless the agent is there, then rides. */
  private void startRide(Ride next, boolean commute) {
    ride = next;
    if (next.byCar()) {
      // A tourist's car is a taxi: nothing is left parked to go back to.
      outByCar = !tourist;
    } else {
      outByTransit = true;
    }
    if (state instanceof PedSimCityActivity activityState) {
      (next.byCar()
              ? (tourist ? activityState.taxiJourneys : activityState.carJourneys)
              : activityState.transitJourneys)
          .increment();
      if (commute) {
        (next.byCar() ? activityState.carCommutes : activityState.transitCommutes).increment();
      }
    }
    if (next.from().equals(originNode)) {
      beginRide();
    } else {
      destinationNode = next.from();
      nextLegRole = next.walkToRole();
      reinitializeMovementPath();
    }
  }

  /** At the boarding stop or the car: off the street until the ride is over. */
  private void beginRide() {
    reachedDestination.set(false);
    updateAgentPosition(ride.from().getCoordinate());
    state.agentsWalking.remove(this);
    pedsim.core.engine.SimulationStateStore.getInstance().removeAgent(agentID);
    status = AgentStatus.AWAY;
    awayUntilStep = state.schedule.getSteps() + ride.minutes() * TimePars.MINUTE_TO_STEPS;
  }

  /** Off the vehicle: walks from the stop or the car park to where the leg was going. */
  @Override
  protected void returnFromAway() {
    Ride done = ride;
    ride = null;
    originNode = done.to();
    destinationNode = done.destination();
    status = done.resumeAs();
    if (done.byCar() && !tourist) {
      parkedAt = done.to().equals(homeNode) ? null : done.to();
    }
    updateAgentPosition(originNode.getCoordinate());
    state.agentsWalking.add(this);
    if (originNode.equals(destinationNode)) {
      reachedDestination.set(true);
      return;
    }
    nextLegRole = done.walkFromRole();
    reinitializeMovementPath();
  }

  /** Each leg starts as what the caller named it, a walk unless it is to or from a stop. */
  @Override
  public void reinitializeMovementPath() {
    legRole = nextLegRole;
    nextLegRole = WALK_LEG;
    super.reinitializeMovementPath();
  }

  @Override
  public String legRole() {
    return legRole;
  }

  /** Reaching the boarding stop or the car starts the ride; anything else is an arrival. */
  @Override
  protected void handleReachedDestination() {
    if (ride != null && !isAway() && ride.from().equals(destinationNode)) {
      beginRide();
      return;
    }
    super.handleReachedDestination();
  }

  private void countOffStreet() {
    if (state instanceof PedSimCityActivity activityState) {
      activityState.unwalkedOffStreet.increment();
    }
  }

  /**
   * Arriving at the workplace is what spends the day's commute, and is when the day's lunch break
   * is settled; arriving back from it resumes the working day. Arriving home for lunch is a stay,
   * not the end of the day's outing.
   */
  @Override
  protected void handleReachedSoloDestination() {
    if (lastDestination != null && lastDestination.equals(workNode)) {
      if (onLunchBreak) {
        onLunchBreak = false;
        lunchAtHome = false;
        status = AgentStatus.AT_DESTINATION;
        timeAtDestination = workEndsAtStep;
        return;
      }
      hasWorkedToday = true;
      super.handleReachedSoloDestination();
      planLunchBreak();
      return;
    }
    if (onLunchBreak && lunchAtHome && homeNode.equals(lastDestination)) {
      state.agentsAtHome.remove(this);
      status = AgentStatus.AT_DESTINATION;
      timeAtDestination =
          state.schedule.getSteps() + lunchHomeStayMinutes * TimePars.MINUTE_TO_STEPS;
      return;
    }
    super.handleReachedSoloDestination();
  }

  /**
   * A worker goes out for lunch with probability {@link ActivityPars#lunchOutShareWorker}, and
   * otherwise goes home for it with probability {@link ActivityPars#lunchHomeShareWorker}, leaving
   * at a minute between {@link ActivityPars#lunchEarliestMinute} and {@link
   * ActivityPars#lunchLatestMinute} when the working day spans the break. Going home needs a
   * resident, and a break of {@link ActivityPars#lunchHomeBreakMinMinutes} to {@link
   * ActivityPars#lunchHomeBreakMaxMinutes} that leaves {@link ActivityPars#lunchStayMinMinutes} at
   * home once the commute is made both ways; otherwise the worker eats at work.
   */
  private void planLunchBreak() {
    if (persona != Persona.WORKER) {
      return;
    }
    double draw = random.nextDouble();
    boolean out = draw < ActivityPars.lunchOutShareWorker;
    boolean home =
        !out
            && !isInbound()
            && draw < ActivityPars.lunchOutShareWorker + ActivityPars.lunchHomeShareWorker;
    if (!out && !home) {
      return;
    }
    int breakMinutes = 60;
    if (home) {
      int breakSpan = ActivityPars.lunchHomeBreakMaxMinutes - ActivityPars.lunchHomeBreakMinMinutes;
      breakMinutes =
          ActivityPars.lunchHomeBreakMinMinutes
              + (breakSpan > 0 ? random.nextInt(breakSpan + 1) : 0);
      int stay = (int) (breakMinutes - 2.0 * commuteMinutes());
      if (stay < ActivityPars.lunchStayMinMinutes) {
        if (state instanceof PedSimCityActivity activityState) {
          activityState.lunchesHomeNoTime.increment();
        }
        return;
      }
      lunchHomeStayMinutes = stay;
    }
    int span = ActivityPars.lunchLatestMinute - ActivityPars.lunchEarliestMinute;
    int minute = ActivityPars.lunchEarliestMinute + (span > 0 ? random.nextInt(span) : 0);
    LocalDateTime now = now();
    int nowMinute = now.getHour() * 60 + now.getMinute();
    double steps = state.schedule.getSteps();
    double lunchStep = steps + (minute - nowMinute) * TimePars.MINUTE_TO_STEPS;
    if (minute <= nowMinute
        || lunchStep + breakMinutes * TimePars.MINUTE_TO_STEPS >= timeAtDestination) {
      return;
    }
    workEndsAtStep = timeAtDestination;
    timeAtDestination = lunchStep;
    lunchPending = true;
    lunchAtHome = home;
    if (state instanceof PedSimCityActivity activityState) {
      (home ? activityState.lunchesHome : activityState.lunchesOut).increment();
    }
  }

  /**
   * Minutes from the workplace home by the commute's mode: walked at the agent's pace; by public
   * transport, the walks to and from stops plus the ride; by car, the walk to the car park plus the
   * drive. Straight lines scaled by the network's circuity, as everywhere a journey is estimated.
   */
  private double commuteMinutes() {
    double walkPerMinute = Pars.moveRate / TimePars.STEP_DURATION * 60.0 * getSpeedFactor();
    switch (commuteMode) {
      case TRANSIT -> {
        Ride ride = Ride.byTransit(workNode, homeNode, status, random);
        if (ride != null) {
          return (walkedMetres(workNode, ride.from()) + walkedMetres(ride.to(), homeNode))
                  / walkPerMinute
              + ride.minutes();
        }
      }
      case CAR_TO_ZONE -> {
        if (commuteCarPark != null) {
          return walkedMetres(workNode, commuteCarPark) / walkPerMinute
              + new Ride(commuteCarPark, homeNode, homeNode, status, true).minutes();
        }
      }
      default -> {}
    }
    return walkedMetres(workNode, homeNode) / walkPerMinute;
  }

  private static double walkedMetres(NodeGraph from, NodeGraph to) {
    return from.getCoordinate().distance(to.getCoordinate()) * Pars.networkCircuityFactor;
  }

  /**
   * Out to a place to eat near the workplace, chosen as any dining destination is, or home by the
   * commute's mode.
   */
  private void startLunchBreak() {
    lunchPending = false;
    onLunchBreak = true;
    currentPurpose = lunchAtHome ? null : ActivityPurpose.DINING;
    if (!startChainedTrip(lunchAtHome ? homeNode : null)) {
      onLunchBreak = false;
      lunchAtHome = false;
      currentPurpose = null;
      status = AgentStatus.AT_DESTINATION;
      timeAtDestination = workEndsAtStep;
    }
  }

  /** Back to work; if the way back is neither walked nor ridden, the agent is simply there. */
  private void returnFromLunchBreak() {
    currentPurpose = null;
    if (!startChainedTrip(workNode)) {
      onLunchBreak = false;
      lunchAtHome = false;
      lastDestination = workNode;
      updateAgentPosition(workNode.getCoordinate());
      status = AgentStatus.AT_DESTINATION;
      timeAtDestination = workEndsAtStep;
    }
  }

  // ----------------------------------------------------------------
  // Trip chain: agenda building and trip chaining
  // ----------------------------------------------------------------

  /** Builds the trip chain agenda when the release manager sends this agent out. */
  @Override
  public void startWalkingAlone() {
    if (tourist && touristDay != null) {
      touristDay.startOuting();
      // The evening's outing is the next departure, from the accommodation.
      mandatoryDepartureMinute = touristDay.eveningMinute();
      agenda = null;
      currentPurpose = null;
      super.startWalkingAlone();
      return;
    }
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
    if (lunchPending) {
      startLunchBreak();
      return;
    }
    if (onLunchBreak) {
      returnFromLunchBreak();
      return;
    }
    ActivityPurpose next =
        tourist && touristDay != null
            ? touristDay.next(minuteOf(now()))
            : (agenda != null) ? agenda.pollOpenActivity(hourOf(now())) : null;
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
    boolean walked = walksLeg(originNode, destinationNode);
    if (tourist) {
      state.agentsWalking.add(this);
      status = AgentStatus.WALKING_ALONE;
      if (destinationNode.getID() == originNode.getID()) {
        reachedDestination.set(true);
      } else if (walked) {
        reinitializeMovementPath();
      } else {
        startTouristRide();
      }
      return true;
    }
    boolean commute = isCommute(originNode, destinationNode);
    boolean byTransit =
        !walked
            && (commute
                ? commuteMode == CommuteMode.TRANSIT
                : ModeChoice.takesTransit(originNode, destinationNode, random));
    boolean byCar =
        !walked
            && !byTransit
            && (commute
                ? commuteMode == CommuteMode.CAR_TO_ZONE
                : ModeChoice.drivesToZone(destinationNode, random));
    if (!walked && !byTransit && !byCar) {
      // The chain ends here; the caller sends the agent home, which is the leg it would have made
      // anyway once its day was over.
      countOffStreet();
      return false;
    }

    state.agentsWalking.add(this);
    status = AgentStatus.WALKING_ALONE;

    if (destinationNode.getID() == originNode.getID()) {
      reachedDestination.set(true);
      return true;
    }
    if (byTransit && startTransitLeg(commute)) {
      return true;
    }
    if (byCar && startCarLeg(commute)) {
      return true;
    }
    reinitializeMovementPath();
    return true;
  }

  /** The trip chain is over: drop the agenda so the next release starts fresh. */
  @Override
  protected void handleReachedHome() {
    agenda = null;
    currentPurpose = null;
    hasWorkedToday = false;
    outByTransit = false;
    outByCar = false;
    parkedAt = null;
    lunchPending = false;
    onLunchBreak = false;
    lunchAtHome = false;
    if (tourist && mandatoryDepartureMinute >= 0) {
      // Back after the evening's outing was due: out again once it has been in a while.
      int soonest = minuteOf(now()) + 15;
      if (mandatoryDepartureMinute < soonest) {
        mandatoryDepartureMinute =
            soonest <= ActivityPars.touristEveningLatestMinute + 60 ? soonest : -1;
      }
    }
    super.handleReachedHome();
  }

  /**
   * Stay durations by activity: the persona's mandatory-stay range at work or study, a plain six
   * to nine hours at a workplace when there is no persona, the purpose's lognormal stay for
   * discretionary stops, and the core uniform draw otherwise.
   */
  @Override
  protected void calculateTimeAtDestination(long steps) {
    boolean atWork = lastDestination != null && lastDestination.equals(workNode);
    if (persona != null && atWork) {
      timeAtDestination =
          persona.sampleMandatoryStayMinutes(random) * TimePars.MINUTE_TO_STEPS + steps;
    } else if (atWork) {
      // No persona to ask, so a working day as anyone would guess it: six to nine hours. Unsourced,
      // and the persona distribution above is the one to prefer.
      timeAtDestination = ((360 + random.nextInt(181)) * TimePars.MINUTE_TO_STEPS) + steps;
    } else if (onLunchBreak) {
      int span = ActivityPars.lunchStayMaxMinutes - ActivityPars.lunchStayMinMinutes;
      int minutes = ActivityPars.lunchStayMinMinutes + (span > 0 ? random.nextInt(span + 1) : 0);
      timeAtDestination = minutes * TimePars.MINUTE_TO_STEPS + steps;
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
   * the trip chain's first leg), then either returns to a place the agent already knows or samples a new
   * one at the released distance, and records the visit either way.
   *
   * <p>The purpose does not scale that distance. It used to; see the note in
   * {@link ActivityPurpose} for why it no longer does.
   */
  @Override
  protected void defineRandomDestination() {
    ensureCurrentPurpose();
    if (ActivityPars.useDestinationChoice) {
      chooseDestination();
      rememberFavourite(currentPurpose, destinationNode);
      return;
    }
    if (tryHabitualDestination()) {
      return;
    }
    super.defineRandomDestination();
    rememberFavourite(currentPurpose, destinationNode);
  }

  /**
   * Resolves the purpose of the trip chain's first discretionary leg from the agenda. Chained legs get
   * their purpose in {@link #goHome()}; agents without persona/agenda keep {@code null} (uniform
   * destination weighting).
   */
  /**
   * Picks a destination from the opportunities around the agent, weighing attraction against
   * distance and habit in one utility. No sampled length is consulted: how far the trip turns out
   * to be is a consequence of the choice, not a target it was made to hit.
   *
   * <p>The choice set is everything within {@link ActivityPars#choiceSetRadiusMetres}, which bounds
   * the work rather than the behaviour - beyond it the impedance term has already made the utility
   * negligible.
   */
  protected void chooseDestination() {
    Graph network = SharedCognitiveMap.getCommunityPrimalNetwork();
    boolean sightseeing = currentPurpose == ActivityPurpose.SIGHTSEEING;
    List<NodeGraph> candidates =
        NodesLookup.getNodesBetweenDistanceInterval(
            network,
            originNode,
            0.0,
            sightseeing
                ? ActivityPars.touristChoiceSetRadiusMetres
                : ActivityPars.choiceSetRadiusMetres);
    Map<NodeGraph, Double> attraction =
        currentPurpose == null ? null : PedSimCityActivity.nodesPurposeWeight.get(currentPurpose);
    if (sightseeing) {
      destinationNode =
          DestinationChoice.choose(
              unseenSights(candidates, attraction), originNode, attraction, null, random);
      if (destinationNode != null) {
        sightsSeen.add(destinationNode);
      }
    } else {
      destinationNode =
          DestinationChoice.choose(
              candidates, originNode, attraction, favouritePlaces.get(currentPurpose), random);
    }
    if (destinationNode == null) {
      state.trace().recordDestinationFallback();
      destinationNode = NodesLookup.randomNode(network, random);
    }
  }

  /**
   * Home, the mandatory activity when there is one, and otherwise the places this persona actually
   * goes.
   *
   * <p>An agent with no workplace anchors instead on what its persona does: a plausible destination
   * for each of its two strongest discretionary purposes, chosen the way the agent would choose on
   * the day rather than taken as the single most attractive node. A retiree's known world is then
   * home, the shops it would use and the errands it would run - which is what an activity space
   * is.
   *
   * <p>Falls back to home alone when the city carries no attraction data, since then there is
   * nothing to anchor on and nothing to invent.
   */
  @Override
  public List<NodeGraph> cognitiveAnchors() {
    List<NodeGraph> anchors = super.cognitiveAnchors();
    if (persona == null || workNode != null || homeNode == null) {
      return anchors;
    }
    Graph network = SharedCognitiveMap.getCommunityPrimalNetwork();
    List<NodeGraph> nearby =
        NodesLookup.getNodesBetweenDistanceInterval(
            network, homeNode, 0.0, ActivityPars.choiceSetRadiusMetres);
    if (nearby.isEmpty()) {
      return anchors;
    }
    List<ActivityPurpose> purposes =
        tourist
            ? List.of(ActivityPurpose.SIGHTSEEING, ActivityPurpose.DINING)
            : persona.strongestPurposes(2);
    for (ActivityPurpose purpose : purposes) {
      Map<NodeGraph, Double> attraction = PedSimCityActivity.nodesPurposeWeight.get(purpose);
      if (attraction == null || attraction.isEmpty()) {
        continue;
      }
      NodeGraph anchor = DestinationChoice.choose(nearby, homeNode, attraction, null, random);
      if (anchor != null && !anchors.contains(anchor)) {
        anchors.add(anchor);
      }
    }
    return anchors;
  }

  protected void ensureCurrentPurpose() {
    if (currentPurpose == null && tourist && touristDay != null) {
      currentPurpose = touristDay.next(minuteOf(now()));
      if (currentPurpose == null) {
        currentPurpose = ActivityPurpose.STROLL;
      }
      return;
    }
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
    if (countFamiliarPlaces() >= ActivityPars.familiarLocationCapacity
        && !evictLeastVisited(node)) {
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

  private static int minuteOf(LocalDateTime time) {
    return time.getHour() * 60 + time.getMinute();
  }

  /**
   * The candidates without the sights this tourist has seen, unless that leaves no sight at all,
   * when it goes back to one.
   */
  private List<NodeGraph> unseenSights(List<NodeGraph> candidates, Map<NodeGraph, Double> sights) {
    if (sights == null || sightsSeen.isEmpty()) {
      return candidates;
    }
    List<NodeGraph> unseen = new ArrayList<>(candidates.size());
    boolean anySight = false;
    for (NodeGraph candidate : candidates) {
      if (!sightsSeen.contains(candidate)) {
        unseen.add(candidate);
        anySight |= sights.getOrDefault(candidate, 0.0) > 0.0;
      }
    }
    return anySight ? unseen : candidates;
  }
}
