package pedsim.activity.engine;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Map;
import java.util.function.Supplier;
import pedsim.activity.agents.ActivityAgent;
import pedsim.activity.agents.ActivityPurpose;
import pedsim.activity.agents.Persona;
import pedsim.activity.agents.WorkplaceChoice;
import pedsim.activity.parameters.ActivityPars;
import pedsim.core.agents.Agent;
import pedsim.core.engine.PedSimCity;
import pedsim.core.engine.Populate;
import pedsim.core.parameters.RouteChoicePars;
import sim.graph.GraphUtils;
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
      activityAgent.setPersona(samplePersona());
      applyPersonaEmployment(activityAgent);
      // Last, because it reads the workplace the line above may have just replaced.
      activityAgent.decideCommuteMode();
    }
  }

  /** Persona draw: zone-conditioned when enabled and the home zone carries census shares. */
  private Persona samplePersona() {
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
