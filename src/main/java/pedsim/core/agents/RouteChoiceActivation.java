package pedsim.core.agents;

/**
 * How likely an agent is to use each route-choice mechanism, as a survey of route choice states it.
 *
 * <p>Hands {@link Heuristics} what it needs to sample a {@link RouteChoiceModel} per trip: whether
 * to use meaningful elements at all; if not, which cost to minimise; if so, which local heuristic
 * routes the legs and which of regions, local landmarks, barrier sub-goals and distant landmarks
 * give the sub-goals.
 *
 * @param usingElements probability of a trip using elements rather than a pure minimisation
 * @param onlyDistance weight of shortest path among pure minimisations
 * @param onlyAngular weight of least angular change among pure minimisations
 * @param localDistance weight of distance as the local heuristic between sub-goals
 * @param localAngular weight of angular change as the local heuristic between sub-goals
 * @param regions probability of region-based navigation
 * @param localLandmarks weight of local-landmark sub-goals
 * @param barrierSubGoals weight of barrier sub-goals
 * @param distantLandmarks probability of landmark-weighted edge costs from distant landmarks
 * @param barriers how the agent perceives barriers when it uses elements
 */
public record RouteChoiceActivation(
    double usingElements,
    double onlyDistance,
    double onlyAngular,
    double localDistance,
    double localAngular,
    double regions,
    double localLandmarks,
    double barrierSubGoals,
    double distantLandmarks,
    RouteChoiceModel.BarrierPreferences barriers) {}
