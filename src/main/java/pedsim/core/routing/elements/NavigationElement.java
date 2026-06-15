package pedsim.core.routing.elements;

import java.util.List;
import sim.graph.NodeGraph;

/** Common contract for all navigation sub-goal strategies. */
public interface NavigationElement {
  List<NodeGraph> computeSequence();
}
