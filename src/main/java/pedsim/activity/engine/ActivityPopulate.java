package pedsim.activity.engine;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.function.Supplier;
import pedsim.activity.agents.ActivityAgent;
import pedsim.activity.agents.ActivityPurpose;
import pedsim.activity.agents.Persona;
import pedsim.activity.agents.WorkplaceChoice;
import pedsim.activity.mobility.CarParks;
import pedsim.activity.mobility.CommuteMode;
import pedsim.activity.mobility.TransitStops;
import pedsim.activity.parameters.ActivityPars;
import pedsim.core.agents.Agent;
import pedsim.core.engine.PedSimCity;
import pedsim.core.engine.Populate;
import pedsim.core.parameters.Pars;
import pedsim.core.parameters.RouteChoicePars;
import pedsim.core.parameters.TimePars;
import pedsim.core.utilities.LoggerUtil;
import sim.graph.NodeGraph;

/**
 * Populate strategy for activity-based modules. Adds census-zone-based residence selection and
 * OSM-tag-based workplace selection on top of core {@link Populate}'s DMA / uniform-random
 * fallbacks.
 *
 * <p>Home nodes are drawn from residence-weighted census zones (the census is population structure
 * only); work nodes are drawn from the WORK-purpose attraction weights the {@link PoiClassifier}
 * derives from OSM-like use tags, with gravity decay from home. When either dataset is missing the
 * behaviour degrades gracefully to the core DMA / random path.
 */
public class ActivityPopulate extends Populate {

  // Residence-weighted sampling table over the residential zones (residence > 0 and ≥1 node).
  private List<CensusZone> residentialZones;
  private double[] cumulativeResidence;
  private double totalResidence = 0.0;

  @Override
  public void populate(PedSimCity state) {
    this.state = state;
    if (!PedSimCityActivity.censusZones.isEmpty()) {
      buildResidenceProbabilities();
    }
    super.populate(state);
    logRealisedSexRatio();
    allocateCarParks(addInboundCommuters());
    addTourists();
  }

  /**
   * Adds the night's tourists ({@link Tourists}), at the same share of the real number as the
   * residents, each sleeping at a node drawn in proportion to its beds. Built last, so every other
   * agent's generator is seeded as it is without them. Their sex is drawn at the city's adult
   * ratio: the accommodation statistics do not record it.
   */
  private void addTourists() {
    if (!Tourists.available()) {
      return;
    }
    double expected =
        Tourists.nightsOn(TimePars.SIMULATION_START_DATE) * Pars.percentagePopulationAgent;
    int count = (int) Math.floor(expected) + (random.nextDouble() < expected % 1.0 ? 1 : 0);
    int nextID = state.agentsList.stream().mapToInt(a -> a.agentID).max().orElse(-1) + 1;
    int firstID = nextID;
    for (int i = 0; i < count; i++) {
      NodeGraph accommodation = Tourists.drawAccommodation(random);
      ActivityAgent agent = createNonResidentAgent(nextID++);
      agent.setHomeAndWorkplace(accommodation, null);
      agent.setFemale(random.nextDouble() < ActivityPars.femaleShare);
      agent.setPersona(Persona.TOURIST);
      agent.makeTourist();
      agent.decideCommuteMode();
      finishNonResidentAgent(agent);
      agent.currentLocation.geometry = GEOMETRY_FACTORY.createPoint(accommodation.getCoordinate());
      state.agents.addGeometry(agent.getLocation());
      agent.updateAgentLists(false, true);
    }
    LoggerUtil.getLogger()
        .info(
            String.format(
                "tourists: %d agents (IDs %d-%d) for %.0f tourist nights on %s",
                count,
                firstID,
                nextID - 1,
                Tourists.nightsOn(TimePars.SIMULATION_START_DATE),
                TimePars.SIMULATION_START_DATE));
  }

  /** An inbound commuter who drives to a workplace in the zone, built once a space is found. */
  protected record CarClaim(InboundCommuters.Flow flow, NodeGraph work) {}

  /**
   * Gives every commuter who drives into the limited-traffic zone a space ({@link
   * CarParks#reserve}), residents and inbound commuters alike, in an order drawn from the seed so
   * that neither group is served first. A resident left without one changes mode ({@link
   * ActivityAgent#loseCommuteCarPark}); an inbound commuter left without one does not come into the
   * city by car, and is not built.
   *
   * @param inbound the inbound commuters waiting for a space
   */
  protected void allocateCarParks(List<CarClaim> inbound) {
    if (!CarParks.available()) {
      return;
    }
    CarParks.scaleTo(Pars.percentagePopulationAgent);
    List<ActivityAgent> residents = new ArrayList<>();
    for (Agent agent : state.agentsList) {
      if (agent instanceof ActivityAgent activityAgent
          && activityAgent.isResident()
          && activityAgent.commuteMode() == CommuteMode.CAR_TO_ZONE) {
        residents.add(activityAgent);
      }
    }
    residents.sort(java.util.Comparator.comparingInt(a -> a.agentID));
    List<Object> claims = new ArrayList<>(residents);
    claims.addAll(inbound);
    for (int i = claims.size() - 1; i > 0; i--) {
      java.util.Collections.swap(claims, i, random.nextInt(i + 1));
    }
    int nextID = state.agentsList.stream().mapToInt(a -> a.agentID).max().orElse(-1) + 1;
    int residentsParked = 0;
    int residentsToTransit = 0;
    int residentsOff = 0;
    int inboundParked = 0;
    int inboundOff = 0;
    for (Object claim : claims) {
      if (claim instanceof ActivityAgent resident) {
        NodeGraph park = CarParks.reserve(resident.workNode, random);
        if (park != null) {
          resident.assignCommuteCarPark(park);
          residentsParked++;
        } else {
          resident.loseCommuteCarPark();
          if (resident.commuteMode() == CommuteMode.TRANSIT) {
            residentsToTransit++;
          } else {
            residentsOff++;
          }
        }
      } else if (claim instanceof CarClaim car) {
        NodeGraph park = CarParks.reserve(car.work(), random);
        if (park == null) {
          inboundOff++;
          continue;
        }
        placeInbound(car.flow(), car.work(), park, nextID++);
        inboundParked++;
      }
    }
    LoggerUtil.getLogger()
        .info(
            String.format(
                "car parks: %d residents and %d inbound commuters drive into the zone; %d residents"
                    + " found no space (%d to public transport, %d off the street), %d inbound"
                    + " commuters did not drive in; %d spaces left for shorter visits",
                residentsParked,
                inboundParked,
                residentsToTransit + residentsOff,
                residentsToTransit,
                residentsOff,
                inboundOff,
                CarParks.freeSpaces()));
  }

  /**
   * Adds the people who come into the city to work or study ({@link InboundCommuters}), at the same
   * share of the real number as the residents: {@code --percentage} of each flow, the remainder
   * drawn. Only those whose journey reaches the street become agents. Each is given a workplace or
   * place of study in proportion to the city's employees or education attraction - from outside
   * the city every part of it is about as far - and arrives where its mode leaves it: the railway
   * station on the shortest line from home to work, or a stop near the workplace.
   */
  private List<CarClaim> addInboundCommuters() {
    List<CarClaim> carClaims = new ArrayList<>();
    List<InboundCommuters.Flow> flows = InboundCommuters.flows();
    if (flows.isEmpty()) {
      return carClaims;
    }
    double share = Pars.percentagePopulationAgent;
    int nextID = state.agentsList.stream().mapToInt(a -> a.agentID).max().orElse(-1) + 1;
    int rail = 0;
    int stop = 0;
    int unplaced = 0;
    double offStreet = 0.0;
    for (InboundCommuters.Flow flow : flows) {
      if (flow.arrival() == InboundCommuters.Arrival.OFF_STREET) {
        offStreet += flow.people();
        continue;
      }
      double expected = flow.people() * share;
      int count = (int) Math.floor(expected) + (random.nextDouble() < expected % 1.0 ? 1 : 0);
      for (int i = 0; i < count; i++) {
        ActivityPurpose purpose = flow.study() ? ActivityPurpose.EDUCATION : ActivityPurpose.WORK;
        java.util.Map<NodeGraph, Double> attraction =
            PedSimCityActivity.nodesPurposeWeight.get(purpose);
        if (attraction == null || attraction.isEmpty()) {
          unplaced++;
          continue;
        }
        NodeGraph anyNode = attraction.keySet().iterator().next();
        NodeGraph work = WorkplaceChoice.draw(anyNode, attraction, 0.0, 0.0, random);
        if (work == null) {
          unplaced++;
          continue;
        }
        if (flow.arrival() == InboundCommuters.Arrival.CAR) {
          if (CarParks.available() && CarParks.inZone(work)) {
            carClaims.add(new CarClaim(flow, work));
          } else {
            offStreet += 1.0 / share;
          }
          continue;
        }
        NodeGraph arrival =
            flow.arrival() == InboundCommuters.Arrival.RAIL
                ? InboundCommuters.stationFor(flow, work)
                : TransitStops.near(work, random);
        if (arrival == null) {
          unplaced++;
          continue;
        }
        placeInbound(flow, work, arrival, nextID++);
        if (flow.arrival() == InboundCommuters.Arrival.RAIL) {
          rail++;
        } else {
          stop++;
        }
      }
    }
    LoggerUtil.getLogger()
        .info(
            String.format(
                "inbound commuters: %d agents (%d by train, %d by tram, metro or bus; %d not"
                    + " placed), %d driving to the limited-traffic zone waiting for a space; %.0f"
                    + " people by car elsewhere or other modes leave no one on the street",
                rail + stop, rail, stop, unplaced, carClaims.size(), offStreet * share));
    return carClaims;
  }

  /** Builds and registers one inbound commuter arriving at {@code arrival}. */
  private void placeInbound(
      InboundCommuters.Flow flow, NodeGraph work, NodeGraph arrival, int agentID) {
    ActivityAgent agent = createNonResidentAgent(agentID);
    agent.setHomeAndWorkplace(arrival, work);
    agent.setFemale(flow.female());
    agent.setPersona(flow.study() ? Persona.STUDENT : Persona.WORKER);
    agent.setInbound(flow);
    agent.decideCommuteMode();
    finishNonResidentAgent(agent);
    agent.currentLocation.geometry = GEOMETRY_FACTORY.createPoint(arrival.getCoordinate());
    state.agents.addGeometry(agent.getLocation());
    agent.updateAgentLists(false, true);
  }

  /** An inbound commuter or tourist of this module's agent class, not yet placed. */
  protected ActivityAgent createNonResidentAgent(int agentID) {
    ActivityAgent agent = new ActivityAgent(this.state, false);
    agent.agentID = agentID;
    return agent;
  }

  /** What a module adds to an inbound commuter or tourist once it is placed; nothing here. */
  protected void finishNonResidentAgent(ActivityAgent agent) {}

  /**
   * Logs the share of the population that is female against the census expectation - the
   * residence-weighted mean of the zones' own rates. The draw is per agent, so the two differ by
   * sampling noise; a systematic gap is a statement about how homes were placed.
   */
  private void logRealisedSexRatio() {
    long women = 0;
    long adults = 0;
    for (Agent agent : state.agentsList) {
      if (agent instanceof ActivityAgent activityAgent && activityAgent.isResident()) {
        adults++;
        if (activityAgent.isFemale()) {
          women++;
        }
      }
    }
    if (adults == 0) {
      return;
    }
    double expected = 0.0;
    double residence = 0.0;
    for (CensusZone zone : PedSimCityActivity.censusZones) {
      if (Double.isNaN(zone.femaleShare) || zone.residence <= 0.0) {
        continue;
      }
      expected += zone.residence * zone.femaleShare;
      residence += zone.residence;
    }
    String census =
        residence > 0.0 ? String.format(" (census %.1f%%)", 100.0 * expected / residence) : "";
    LoggerUtil.getLogger()
        .info(String.format("women: %.1f%% of agents%s", 100.0 * women / adults, census));
  }

  @Override
  protected Agent createAgent(int agentID) {
    ActivityAgent agent = new ActivityAgent(this.state, false);
    agent.agentID = agentID;
    defineHomeWorkLocations(agent);
    return agent;
  }

  // Zone the current agent's home was drawn from; null when home did not come from the census.
  private CensusZone homeZone;

  /**
   * Assigns home/work, then samples the persona — conditioned on the home zone's age structure
   * when the census provides it — and applies its employment status: personas without a mandatory
   * activity get no work node (no daily commute); students are re-targeted to an education-tagged
   * node when the city provides them. Finally it settles whether that commute is walked at all, from
   * how far it turned out to be. Runs for every activity-based agent, including night agents
   * (their populate goes through this method too).
   */
  @Override
  protected void defineHomeWorkLocations(Agent agent) {
    homeZone = null;
    super.defineHomeWorkLocations(agent);
    if (agent instanceof ActivityAgent activityAgent) {
      activityAgent.setFemale(random.nextDouble() < femaleShare());
      activityAgent.setPersona(samplePersona());
      applyPersonaEmployment(activityAgent);
      // Last, because it reads the workplace the line above may have just replaced.
      activityAgent.decideCommuteMode();
    }
  }

  /**
   * The home zone's share of adults who are women, falling back to {@link ActivityPars#femaleShare}
   * where the census carries no such column or the home node came from a rung below the census.
   */
  private double femaleShare() {
    if (homeZone == null || Double.isNaN(homeZone.femaleShare)) {
      return ActivityPars.femaleShare;
    }
    return Math.min(1.0, Math.max(0.0, homeZone.femaleShare));
  }

  /** Persona draw: zone-conditioned when enabled and the home zone carries census shares. */
  protected Persona samplePersona() {
    if (ActivityPars.useCensusPersonas && homeZone != null) {
      return Persona.sample(
          random, homeZone.retireeShare, homeZone.studentShare, homeZone.workerShare);
    }
    return Persona.sample(random);
  }

  private void applyPersonaEmployment(ActivityAgent agent) {
    Persona persona = agent.getPersona();
    if (persona == null) {
      return;
    }
    if (!persona.hasMandatoryActivity()) {
      // Retirees / flex adults have no daily commute; all their trips are discretionary.
      agent.setHomeAndWorkplace(agent.getHome(), null);
      return;
    }
    if (persona == Persona.STUDENT) {
      NodeGraph studyNode = sampleEducationNode(agent.getHome());
      if (studyNode != null) {
        agent.setHomeAndWorkplace(agent.getHome(), studyNode);
      }
    }
  }

  /**
   * Weighted draw among education-tagged nodes, with the same gravity decay as the workplace draw,
   * or {@code null} when the city has no education tags (the workplace assignment then stands in).
   *
   * <p>Uncapped for the same reason as the workplace: a school out of walking range is a school
   * reached some other way, not a school the student does not attend. The decay replaces the cap —
   * without it, removing the bound would scatter students across the whole city by attraction
   * alone.
   */
  private NodeGraph sampleEducationNode(NodeGraph homeNode) {
    return WorkplaceChoice.draw(
        homeNode,
        PedSimCityActivity.nodesPurposeWeight.get(ActivityPurpose.EDUCATION),
        ActivityPars.educationDistanceDecay,
        ActivityPars.workplaceMinDistanceMetres,
        random);
  }

  /** Census zones first, then core's ladder. The uniform draw that ends it is core's. */
  @Override
  protected List<Supplier<NodeGraph>> residenceLadder() {
    List<Supplier<NodeGraph>> ladder = new ArrayList<>();
    ladder.add(this::selectHomeNodeFromCensus);
    // DMA only when census data was attempted and its zone lookup returned no node - so a city
    // with no census zones at all goes straight to the uniform draw, skipping the DMA rung core
    // would have used. That is how this has always behaved; it is not obviously what is wanted.
    ladder.add(() -> hasUsableCensusZones() ? selectHomeNodeWithDMA() : null);
    return ladder;
  }

  /** A residence-weighted census zone, then a node inside it. Null when the census cannot say. */
  private NodeGraph selectHomeNodeFromCensus() {
    if (!hasUsableCensusZones()) {
      return null;
    }
    CensusZone zone = sampleResidentialZone();
    NodeGraph node = randomNodeIn(zone);
    if (node != null) {
      homeZone = zone; // remembered so the persona can be conditioned on the home zone
    }
    return node;
  }

  /** OSM purpose weights first, then core's ladder. The uniform draw that ends it is core's. */
  @Override
  protected List<Supplier<NodeGraph>> workplaceLadder() {
    List<Supplier<NodeGraph>> ladder = new ArrayList<>();
    ladder.add(() -> selectWorkNodeFromPurposeWeights(homeNode));
    ladder.addAll(super.workplaceLadder());
    return ladder;
  }

  /** Builds the cumulative residence-weight table over the residential zones (residence &gt; 0). */
  private void buildResidenceProbabilities() {
    residentialZones = new ArrayList<>();
    for (CensusZone zone : PedSimCityActivity.censusZones) {
      if (zone.residence > 0.0 && !zone.nodes.isEmpty()) {
        residentialZones.add(zone);
      }
    }
    cumulativeResidence = new double[residentialZones.size()];
    double cumulative = 0.0;
    totalResidence = 0.0;
    for (int i = 0; i < residentialZones.size(); i++) {
      cumulative += residentialZones.get(i).residence;
      cumulativeResidence[i] = cumulative;
      totalResidence += residentialZones.get(i).residence;
    }
  }

  private boolean hasUsableCensusZones() {
    return residentialZones != null && !residentialZones.isEmpty() && totalResidence > 0.0;
  }

  private CensusZone sampleResidentialZone() {
    double r = random.nextDouble() * totalResidence;
    int idx = Arrays.binarySearch(cumulativeResidence, r);
    if (idx < 0) idx = -idx - 1; // insertion point = first zone whose cumulative weight exceeds r
    if (idx >= residentialZones.size()) idx = residentialZones.size() - 1;
    return residentialZones.get(idx);
  }

  private NodeGraph randomNodeIn(CensusZone zone) {
    if (zone == null || zone.nodes.isEmpty()) return null;
    return zone.nodes.get(random.nextInt(zone.nodes.size()));
  }

  /**
   * Weighted draw among WORK-tagged nodes (offices, commercial, industrial… from the OSM-tag
   * purpose weights), with gravity decay when {@link RouteChoicePars#useGravityModel} is set.
   * Returns {@code null} when the city carries no work tags — the DMA / distance fallbacks then
   * stand in.
   *
   * <p>There is no upper bound on the commute. A workplace is where it is; what decides whether
   * that commute appears in this model is {@link
   * pedsim.activity.agents.ActivityAgent#decideCommuteMode()}, and a commute too long to walk
   * should surface as the walk to a transit stop rather than as a workplace the city never
   * assigned. The old cap placed every workplace inside the discretionary trip range, which made
   * the long commute — and therefore the pedestrian volume around stations — impossible to
   * represent at all.
   */
  private NodeGraph selectWorkNodeFromPurposeWeights(NodeGraph homeNode) {
    return WorkplaceChoice.draw(
        homeNode,
        PedSimCityActivity.nodesPurposeWeight.get(ActivityPurpose.WORK),
        ActivityPars.workplaceDistanceDecay,
        ActivityPars.workplaceMinDistanceMetres,
        random);
  }
}
