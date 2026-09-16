/**
 * Route assembly: a whole {@link sim.routing.Route} for one route-choice model.
 *
 * <p>A router turns "this agent, this model, this origin and destination" into a finished route. It
 * chooses which search to run and runs it once per leg when the model produces sub-goals, corrects
 * the direction of edges a search returned reversed, backtracks when a leg fails, widens to the
 * full network when an individualised agent's known one cannot connect the pair, and falls back to
 * the shortest path when nothing else answers - recording each of those in {@code RouteTrace}, so a
 * run can say how often it happened rather than presenting a substitute as the model's own route.
 *
 * <p>Below it is {@link pedsim.core.routing.search}: one search between two nodes, which knows
 * nothing about models. The sub-goals come from {@link pedsim.core.routing.elements}. Above it,
 * {@link pedsim.core.routing.RoutePlanner} reads {@code AgentProperties} and picks the router.
 *
 * <p>The classes here are named {@code *PathFinder}; a path finder in this sense is a router, not a
 * search.
 */
package pedsim.core.routing.routers;
