package pedsim.core.agents;

import java.util.ArrayList;
import java.util.List;
import java.util.Set;
import org.locationtech.jts.geom.Coordinate;
import org.locationtech.jts.geom.LineString;
import org.locationtech.jts.linearref.LengthIndexedLine;
import org.locationtech.jts.planargraph.DirectedEdge;
import pedsim.core.engine.PedSimCity;
import pedsim.core.parameters.Pars;
import sim.graph.EdgeGraph;
import sim.graph.NodeGraph;
import sim.routing.Route;

public class AgentMovement {

  // How much to move the agent by in each step()
  protected double reach = 0.0;

  // start, current, end position along current line
  protected DirectedEdge firstDirectedEdge = null;
  public EdgeGraph currentEdge = null;
  protected DirectedEdge currentDirectedEdge = null;
  protected double currentIndex = 0.0;
  protected double endIndex = 0.0;

  // used by agent to walk along line segment
  protected int indexOnSequence = 0;
  protected LengthIndexedLine indexedSegment = null;
  protected List<DirectedEdge> directedEdgesSequence = new ArrayList<>();
  protected Agent agent;
  protected List<DirectedEdge> edgesWalkedSoFar = new ArrayList<>();
  protected Set<EdgeGraph> edgesToAvoid;

  PedSimCity state;

  protected NodeGraph currentNode;

  public AgentMovement(Agent agent) {
    this.agent = agent;
    this.state = agent.getState();
  }

  public AgentMovement() {}

  /**
   * Initialises the directedEdgesSequence (the path) for the agent.
   */
  public void initialisePath(Route route) {

    indexOnSequence = 0;
    this.directedEdgesSequence = route.directedEdgesSequence;

    // set up how to traverse this first link
    firstDirectedEdge = directedEdgesSequence.get(indexOnSequence);
    currentNode = (NodeGraph) firstDirectedEdge.getFromNode();
    agent.updateAgentPosition(currentNode.getCoordinate());
    // Sets the Agent up to proceed along an Edge
    setupEdge(firstDirectedEdge);
  }

  /**
   * Sets the agent up to proceed along a specified edge.
   *
   * @param directedEdge The DirectedEdge to traverse next.
   */
  protected void setupEdge(DirectedEdge directedEdge) {

    currentDirectedEdge = directedEdge;
    currentEdge = (EdgeGraph) currentDirectedEdge.getEdge();

    // if (isEdgeCrowded(currentEdge)) {
    // rerouteOrIncreaseSpeed();
    // }
    updateCounts();

    if (PedSimCity.indexedEdgeCache.containsKey(currentDirectedEdge)) {
      indexedSegment = PedSimCity.indexedEdgeCache.get(currentDirectedEdge);
    } else {
      addIndexedSegment(currentEdge);
      indexedSegment = PedSimCity.indexedEdgeCache.get(currentDirectedEdge);
    }

    currentIndex = indexedSegment.getStartIndex();
    endIndex = indexedSegment.getEndIndex();
    return;
  }

  /**
   * Moves the agent along the current path.
   */
  public void keepWalking() {

    resetReach(); // as the segment might have changed level of crowdness
    // updateReach();
    // move along the current segment
    currentIndex += reach;

    // check to see if the progress has taken the current index beyond its goal
    // If so, proceed to the next edge
    if (currentIndex > endIndex) {
      final Coordinate currentPos = indexedSegment.extractPoint(endIndex);
      agent.updateAgentPosition(currentPos);
      double residualMove = currentIndex - endIndex;
      transitionToNextEdge(residualMove);
    } else {
      // just update the position!
      final Coordinate currentPos = indexedSegment.extractPoint(currentIndex);
      agent.updateAgentPosition(currentPos);
    }
  }

  /**
   * Transitions to the next edge in the {@code directedEdgesSequence}.
   *
   * @param residualMove The amount of distance the agent can still travel this step.
   */
  protected void transitionToNextEdge(double residualMove) {

    // update the counter for where the index on the directedEdgesSequence is
    indexOnSequence += 1;
    currentEdge.decrementAgentCount(); // Leave current edge

    // check to make sure the Agent has not reached the end of the
    // directedEdgesSequence already
    if (indexOnSequence >= directedEdgesSequence.size()) {
      agent.reachedDestination.set(true);
      indexOnSequence -= 1; // make sure index is correct
      updateData();
      return;
    }

    // prepare to setup to the next edge
    DirectedEdge nextDirectedEdge = directedEdgesSequence.get(indexOnSequence);
    setupEdge(nextDirectedEdge);

    reach = residualMove;
    currentIndex += reach;

    // check to see if the progress has taken the current index beyond its goal
    // given the direction of movement. If so, proceed to the next edge
    if (currentIndex > endIndex) {
      residualMove = currentIndex - endIndex;
      transitionToNextEdge(residualMove);
    }
  }

  /**
   * Adds an indexed segment to the indexed edge cache.
   *
   * @param edge The edge to add to the indexed edge cache.
   */
  protected void addIndexedSegment(EdgeGraph edge) {

    LineString line = edge.getLine();
    double distanceToStart = line.getStartPoint().distance(agent.getLocation().geometry);
    double distanceToEnd = line.getEndPoint().distance(agent.getLocation().geometry);

    if (distanceToEnd < distanceToStart) {
      line = line.reverse();
    }

    final LineString finalLine = line;
    PedSimCity.indexedEdgeCache.put(currentDirectedEdge, new LengthIndexedLine(finalLine));
  }

  /**
   * Resets the agent's movement reach to the base move rate.
   */
  protected void resetReach() {
    reach = Pars.moveRate;
  }

  /**
   * Increases the agent's movement reach based on the speed factor for night time.
   */
  protected void increaseReach() {
    reach = reach + (Pars.moveRate * Pars.SPEED_INCREMENT_FACTOR);
  }

  /**
   * Updates the counts for the current edge the agent is walking on.
   */
  protected void updateCounts() {
    edgesWalkedSoFar.add(currentDirectedEdge);
    currentEdge.incrementAgentCount();
    agent.metersWalkedTot += currentEdge.getLength();
    agent.metersWalkedDay += currentEdge.getLength();
  }

  /**
   * Updates the data related to the agent's route to derive pedestrian volumes.
   */
  public void updateData() {
    agent.getRoute().resetRoute(new ArrayList<>(edgesWalkedSoFar));
    state.flowHandler.updateFlowsData(agent, agent.getRoute(), agent.getAgentScenario(), null);
  }

  /**
   * Resets the path for the agent to follow a new sequence of directed edges.
   *
   * @param directedEdgesSequence The new sequence of directed edges.
   */
  protected void resetPath(List<DirectedEdge> directedEdgesSequence) {

    indexOnSequence = 0;
    this.directedEdgesSequence = directedEdgesSequence;
    currentDirectedEdge = directedEdgesSequence.get(0);

    // set up how to traverse this first link
    currentDirectedEdge = directedEdgesSequence.get(indexOnSequence);
    currentEdge = (EdgeGraph) currentDirectedEdge.getEdge();
    currentNode = (NodeGraph) firstDirectedEdge.getFromNode();
    edgesToAvoid.clear();
    agent.updateAgentPosition(currentNode.getCoordinate());
  }
}
