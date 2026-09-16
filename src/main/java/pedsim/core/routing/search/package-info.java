/**
 * Graph search: one path between two nodes, at one cost.
 *
 * <p>{@link pedsim.core.routing.search.Dijkstra} and its subclasses answer a single question - the
 * cheapest sequence of directed edges from an origin to a destination, under the cost the subclass
 * defines: metric length, angular change, or global landmarkness. They know about the graph, what
 * the agent knows of it, the perception error and the nodes and edges to avoid. They know nothing
 * about route-choice models, sub-goals, or what to do when there is no path.
 *
 * <p>The tier above is {@link pedsim.core.routing.routers}, which decides which search to run and
 * how many times; above that, {@link pedsim.core.routing.RoutePlanner} reads the agent's properties
 * and picks the router.
 */
package pedsim.core.routing.search;
