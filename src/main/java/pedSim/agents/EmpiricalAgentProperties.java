package pedSim.agents;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Random;

import org.javatuples.Pair;

import pedSim.utilities.StringEnum.BarrierType;
import pedSim.utilities.StringEnum.Groups;
import pedSim.utilities.StringEnum.LandmarkType;
import pedSim.utilities.StringEnum.RouteChoiceProperty;
import sim.util.geo.Utilities;

/**
 * `EmpiricalAgentProperties` is a subclass of `AgentProperties` that represents
 * the properties of an agent in a pedestrian simulation with empirical-based
 * parameters. It extends the base `AgentProperties` class to incorporate
 * additional parameters.
 */
public class EmpiricalAgentProperties extends AgentProperties {

	public Groups groupName;
	EmpiricalAgentsGroup group;
	boolean usingElements = false;
	boolean elementsActivated = false;

	List<Double> elementsProbability = new ArrayList<>(Arrays.asList(0.0, 0.0));
	List<Double> minimisationProbability = new ArrayList<>(Arrays.asList(0.0, 0.0));
	List<Double> localHeuristicsProbability = new ArrayList<>(Arrays.asList(0.0, 0.0));
	List<Double> regionBasedProbability = new ArrayList<>(Arrays.asList(0.0));
	List<Double> subGoalsProbability = new ArrayList<>(Arrays.asList(0.0, 0.0));
	List<Double> distantLandmarksProbability = new ArrayList<>(Arrays.asList(0.0));

	Map<RouteChoiceProperty, Double> elementsMap = new HashMap<>();
	Map<RouteChoiceProperty, Double> minimisationMap = new HashMap<>();
	Map<RouteChoiceProperty, Double> localHeuristicsMap = new HashMap<>();
	Map<RouteChoiceProperty, Double> regionBasedMap = new HashMap<>();
	Map<RouteChoiceProperty, Double> subGoalsMap = new HashMap<>();
	Map<RouteChoiceProperty, Double> distantLandmarksMap = new HashMap<>();
	Map<RouteChoiceProperty, Double> randomElementsMap = new HashMap<>();

	List<RouteChoiceProperty> elements = new ArrayList<>(
			Arrays.asList(RouteChoiceProperty.USING_ELEMENTS, RouteChoiceProperty.NOT_USING_ELEMENTS));
	List<RouteChoiceProperty> minimisation = new ArrayList<>(
			Arrays.asList(RouteChoiceProperty.ROAD_DISTANCE, RouteChoiceProperty.ANGULAR_CHANGE));
	List<RouteChoiceProperty> localHeuristics = new ArrayList<>(
			Arrays.asList(RouteChoiceProperty.ROAD_DISTANCE_LOCAL, RouteChoiceProperty.ANGULAR_CHANGE_LOCAL));
	List<RouteChoiceProperty> subGoals = new ArrayList<>(Arrays.asList(RouteChoiceProperty.LOCAL_LANDMARKS,
			RouteChoiceProperty.BARRIER_SUBGOALS, RouteChoiceProperty.NO_SUBGOALS));
	List<RouteChoiceProperty> regionBased = new ArrayList<>(
			Arrays.asList(RouteChoiceProperty.REGION_BASED, RouteChoiceProperty.NOT_REGION_BASED));
	List<RouteChoiceProperty> distantLandmarks = new ArrayList<>(
			Arrays.asList(RouteChoiceProperty.USING_DISTANT, RouteChoiceProperty.NOT_USING_DISTANT));
	List<RouteChoiceProperty> randomElements = new ArrayList<>(
			Arrays.asList(RouteChoiceProperty.REGION_BASED, RouteChoiceProperty.LOCAL_LANDMARKS,
					RouteChoiceProperty.BARRIER_SUBGOALS, RouteChoiceProperty.USING_DISTANT));

	// Constants for magic numbers and strings
	private static final double MIN_NATURAL_BARRIERS = 1.00;
	private static final double MAX_NATURAL_BARRIERS = 0.00;
	private static final double MIN_SEVERING_BARRIERS = 1.00;
	private static final double MAX_SEVERING_BARRIERS = 2.00;

	/**
	 * Constructs an instance of EmpiricalAgentProperties for an agent, initializing
	 * it with properties from the specified EmpiricalAgentsGroup.
	 *
	 * @param agent The agent for which properties are being set.
	 * @param group The EmpiricalAgentsGroup containing the properties to initialize
	 *              this agent's properties.
	 */
	public EmpiricalAgentProperties(Agent agent, EmpiricalAgentsGroup group) {
		super();
		this.group = group;
		this.groupName = this.group.groupName;
	}

	/**
	 * Sets route choice parameters for the agent based on a group's properties.
	 * This method updates the agent's route choice probabilities and properties
	 * according to the properties defined for the agent's group.
	 */
	public void setParametersFromGroup() {
		if (groupName.equals(Groups.NULLGROUP))
			return;

		// minimisation
		Pair<Double, Double> probabilityUsingElements = new Pair<>(group.probabilityUsingElements,
				group.probabilityUsingElementsSD);
		Pair<Double, Double> probabilityNotUsingElements = new Pair<>(group.probabilityNotUsingElements,
				group.probabilityNotUsingElementsSD);
		ArrayList<Pair<Double, Double>> elementsDis = new ArrayList<>(
				Arrays.asList(probabilityUsingElements, probabilityNotUsingElements));
		updateProbabilities(elementsProbability, elementsDis);
		mapProbabilities(elements, elementsProbability, elementsMap);

		// only minimisation
		Pair<Double, Double> probabilityOnlyRoadDistance = new Pair<>(group.probabilityRoadDistance,
				group.probabilityRoadDistanceSD);
		Pair<Double, Double> probabilityOnlyAngularChange = new Pair<>(group.probabilityAngularChange,
				group.probabilityAngularChangeSD);
		ArrayList<Pair<Double, Double>> onlyMinimisationDis = new ArrayList<>(
				Arrays.asList(probabilityOnlyRoadDistance, probabilityOnlyAngularChange));
		updateProbabilities(minimisationProbability, onlyMinimisationDis);
		mapProbabilities(minimisation, minimisationProbability, minimisationMap);

		// heuristics
		Pair<Double, Double> probabilityRoadDistance = new Pair<>(group.probabilityLocalRoadDistance,
				group.probabilityLocalRoadDistanceSD);
		Pair<Double, Double> probabilityAngularChange = new Pair<>(group.probabilityLocalAngularChange,
				group.probabilityLocalAngularChangeSD);
		ArrayList<Pair<Double, Double>> localHeuristicsDis = new ArrayList<>(
				Arrays.asList(probabilityRoadDistance, probabilityAngularChange));
		updateProbabilities(localHeuristicsProbability, localHeuristicsDis);
		mapProbabilities(localHeuristics, localHeuristicsProbability, localHeuristicsMap);

		// Coarse plan
		Pair<Double, Double> probabilityRegionBased = new Pair<>(group.probabilityRegionBasedNavigation,
				group.probabilityRegionBasedNavigationSD);
		final ArrayList<Pair<Double, Double>> pRegionsDis = new ArrayList<>(Arrays.asList(probabilityRegionBased));
		updateProbabilities(regionBasedProbability, pRegionsDis);
		for (RouteChoiceProperty region : regionBased) {
			double p = region.equals(RouteChoiceProperty.NOT_REGION_BASED) ? 1.0 - regionBasedProbability.get(0)
					: regionBasedProbability.get(0);
			regionBasedMap.put(region, p);
		}

		// subgoals
		Pair<Double, Double> probabilityLocalLandmarks = new Pair<>(group.probabilityLocalLandmarks,
				group.probabilityLocalLandmarksSD);
		Pair<Double, Double> probabilityBarrierSubGoals = new Pair<>(group.probabilityBarrierSubGoals,
				group.probabilityBarrierSubGoalsSD);
		ArrayList<Pair<Double, Double>> pSubGoalsDis = new ArrayList<>(
				Arrays.asList(probabilityLocalLandmarks, probabilityBarrierSubGoals));

		updateProbabilities(subGoalsProbability, pSubGoalsDis);
		for (RouteChoiceProperty subGoal : subGoals) {
			double p = subGoal.equals(RouteChoiceProperty.LOCAL_LANDMARKS)
					? subGoalsProbability.get(subGoals.indexOf(subGoal))
					: 1.00 - subGoalsProbability.stream().mapToDouble(d -> d).sum();
			subGoalsMap.put(subGoal, p);
		}

		// distant landmarks
		Pair<Double, Double> pDistantLandmarksTmp = new Pair<>(group.probabilityDistantLandmarks,
				group.probabilityDistantLandmarksSD);
		ArrayList<Pair<Double, Double>> distantLandmarksDis = new ArrayList<>(Arrays.asList(pDistantLandmarksTmp));
		updateProbabilities(distantLandmarksProbability, distantLandmarksDis);

		for (RouteChoiceProperty landmark : distantLandmarks) {
			double p = landmark.equals(RouteChoiceProperty.NOT_USING_DISTANT)
					? 1.00 - distantLandmarksProbability.get(0)
					: distantLandmarksProbability.get(0);
			distantLandmarksMap.put(landmark, p);
		}

		// other elements
		randomElementsMap.put(randomElements.get(0), regionBasedProbability.get(0));
		randomElementsMap.put(randomElements.get(1), subGoalsProbability.get(0));
		randomElementsMap.put(randomElements.get(2), subGoalsProbability.get(1));
		randomElementsMap.put(randomElements.get(3), distantLandmarksProbability.get(0));

		// other route properties
		naturalBarriers = rescale(0.0, 1.0, MAX_NATURAL_BARRIERS, MIN_NATURAL_BARRIERS, group.naturalBarriers);
		this.naturalBarriersSD = group.naturalBarriersSD;
		severingBarriers = rescale(0.0, 1.0, MIN_SEVERING_BARRIERS, MAX_SEVERING_BARRIERS, group.severingBarriers);
		this.severingBarriersSD = group.severingBarriersSD;
	}

	/**
	 * Updates the probabilities of a list of values based on a probability
	 * distribution.
	 *
	 * @param probabilities The list of probabilities to update.
	 * @param pDistribution The probability distribution as a list of pairs
	 *                      representing mean and standard deviation.
	 */
	public void updateProbabilities(List<Double> probabilities, List<Pair<Double, Double>> pDistribution) {
		for (final Double d : probabilities) {
			final int index = probabilities.indexOf(d);
			final double p = Utilities.fromDistribution(pDistribution.get(index).getValue0(),
					pDistribution.get(index).getValue1(), null);
			probabilities.set(index, p);
		}
	}

	/**
	 * Maps a list of properties to their corresponding probabilities and stores
	 * them in a map.
	 *
	 * @param properties            The list of properties to map.
	 * @param propertiesProbability The list of probabilities associated with each
	 *                              property.
	 * @param propertiesMap         The map to store the properties and their
	 *                              probabilities.
	 */
	private void mapProbabilities(List<RouteChoiceProperty> properties, List<Double> propertiesProbability,
			Map<RouteChoiceProperty, Double> propertiesMap) {

		for (final RouteChoiceProperty property : properties) {
			final int index = properties.indexOf(property);
			final double probability = propertiesProbability.get(index);
			propertiesMap.put(property, probability);
		}
	}

	/**
	 * Randomly assigns route choice parameters to the agent based on its route
	 * choice group and other settings. This method initialises various route choice
	 * properties such as minimisation approaches, use of elements, and local
	 * minimisation heuristics based on probability distributions specified in the
	 * group and settings.
	 */
	public void randomizeRouteChoiceParameters() {
		reset();

		if (this.groupName.equals(Groups.NULLGROUP))
			fromUniform();
		else
			setParametersFromGroup();

		final Random random = new Random();

		// using elements or not
		List<RouteChoiceProperty> keys = new ArrayList<>(elementsMap.keySet());
		double pRandom = random.nextDouble() * elementsMap.values().stream().mapToDouble(d -> d).sum();
		double limit = 0.0;
		for (final RouteChoiceProperty key : keys) {
			double p = elementsMap.get(key);
			if (pRandom <= p + limit) {
				usingElements = key.equals(RouteChoiceProperty.USING_ELEMENTS);
				break;
			}
			limit += p;
		}

		// minimisation approaches
		if (!usingElements) {
			keys.clear();
			keys = new ArrayList<>(minimisationMap.keySet());
			pRandom = random.nextDouble() * minimisationMap.values().stream().mapToDouble(d -> d).sum();
			limit = 0.0;
			for (final RouteChoiceProperty key : keys) {
				final double p = minimisationMap.get(key);
				if (pRandom <= p + limit) {
					minimisingDistance = key.equals(RouteChoiceProperty.ROAD_DISTANCE);
					minimisingAngular = !minimisingDistance;
					break;
				}
				limit += p;
			}
			return;
		}

		if (naturalBarriers < 0.95)
			preferenceNaturalBarriers = true;
		if (severingBarriers > 1.05)
			aversionSeveringBarriers = true;

		// local minimisation heuristic
		keys.clear();
		keys = new ArrayList<>(localHeuristicsMap.keySet());
		pRandom = random.nextDouble() * localHeuristicsMap.values().stream().mapToDouble(d -> d).sum();
		limit = 0.0;
		for (final RouteChoiceProperty key : keys) {
			final double p = localHeuristicsMap.get(key);
			if (pRandom <= p + limit) {
				localHeuristicDistance = key.equals(RouteChoiceProperty.ROAD_DISTANCE_LOCAL);
				localHeuristicAngular = !localHeuristicDistance;
				break;
			}
			limit += p;
		}

		while (usingElements && !elementsActivated)
			activateElements();
	}

	/**
	 * Resets all route choice properties and related flags to their default states.
	 * This method is used to clear any previously assigned route choice parameters.
	 */
	public void reset() {
		usingElements = false;
		elementsActivated = false;
		onlyMinimising = false;
		minimisingDistance = false;
		minimisingAngular = false;
		localHeuristicDistance = false;
		localHeuristicAngular = false;
		barrierType = null;
		usingLocalLandmarks = false;
		barrierBasedNavigation = false;
		regionBasedNavigation = false;
		usingDistantLandmarks = false;
		preferenceNaturalBarriers = false;
		aversionSeveringBarriers = false;
	}

	/**
	 * Initialises route choice probabilities uniformly for various route choice
	 * properties. This method assigns equal probabilities to all available choices
	 * for each route choice property. It is used when the agent belongs to the
	 * "nullGroup" or when route choice settings are uniform.
	 */
	private void fromUniform() {
		initializeUniformProbabilities(elements, elementsMap);
		initializeUniformProbabilities(minimisation, minimisationMap);
		initializeUniformProbabilities(localHeuristics, localHeuristicsMap);
		initializeUniformProbabilities(regionBased, regionBasedMap);
		initializeUniformProbabilities(subGoals, subGoalsMap);
		initializeUniformProbabilities(distantLandmarks, distantLandmarksMap);

		naturalBarriers = 0.00 + Math.random() * (1.00 - 0.00);
		severingBarriers = 1.00 + Math.random() * (2.00 - 1.00);
	}

	/**
	 * Rescales a given value from an old range to a new range.
	 *
	 * @param oldMin The minimum value of the old range.
	 * @param oldMax The maximum value of the old range.
	 * @param newMin The minimum value of the new range.
	 * @param newMax The maximum value of the new range.
	 * @param value  The value to be rescaled.
	 * @return The rescaled value within the new range.
	 */
	private double rescale(double oldMin, double oldMax, double newMin, double newMax, double value) {
		double oldRange = oldMax - oldMin;
		double newRange = newMax - newMin;
		return (value - oldMin) * newRange / oldRange + newMin;
	}

	/**
	 * Initialises route choice probabilities uniformly for a list of route choice
	 * properties. This method assigns equal probabilities to all available choices
	 * for each route choice property.
	 *
	 * @param properties  The list of route choice properties to initialise
	 *                    probabilities for.
	 * @param propertyMap The map to store the initialised probabilities for each
	 *                    property.
	 */
	private void initializeUniformProbabilities(List<RouteChoiceProperty> properties,
			Map<RouteChoiceProperty, Double> propertyMap) {
		final double probability = 1.0 / Double.valueOf(properties.size());
		for (final RouteChoiceProperty property : properties)
			propertyMap.put(property, probability);
	}

	/**
	 * Activates route choice elements based on randomised probabilities. This
	 * method randomly activates route choice elements (e.g., region-based,
	 * subgoals, global landmarks) based on the specified probabilities, thereby
	 * affecting the agent's route choice behaviour.
	 */
	private void activateElements() {
		final Random random = new Random();
		List<RouteChoiceProperty> keys = new ArrayList<>(regionBasedMap.keySet());

		double pRandom = random.nextDouble() * regionBasedMap.values().stream().mapToDouble(d -> d).sum();
		double limit = 0.0;
		for (final RouteChoiceProperty key : keys) {
			final double p = regionBasedMap.get(key);
			if (pRandom <= p + limit) {
				regionBasedNavigation = key.equals(RouteChoiceProperty.REGION_BASED);
				elementsActivated = true;
				break;
			}
			limit += p;
		}

		// subgoals
		keys.clear();
		keys = new ArrayList<>(subGoalsMap.keySet());
		pRandom = random.nextDouble() * subGoalsMap.values().stream().mapToDouble(d -> d).sum();
		limit = 0.0;
		for (final RouteChoiceProperty key : keys) {
			final double p = subGoalsMap.get(key);
			if (pRandom <= p + limit) {
				usingLocalLandmarks = key.equals(RouteChoiceProperty.LOCAL_LANDMARKS);
				barrierBasedNavigation = key.equals(RouteChoiceProperty.BARRIER_SUBGOALS);
				elementsActivated = true;
				barrierType = BarrierType.SEPARATING;
				landmarkType = LandmarkType.LOCAL;
				break;
			}
			limit += p;
		}

		// global landmarks
		keys.clear();
		keys = new ArrayList<>(distantLandmarksMap.keySet());
		pRandom = random.nextDouble() * distantLandmarksMap.values().stream().mapToDouble(d -> d).sum();
		limit = 0.0;
		for (final RouteChoiceProperty key : keys) {
			final double p = distantLandmarksMap.get(key);
			if (pRandom <= p + limit) {
				elementsActivated = true;
				usingDistantLandmarks = key.equals(RouteChoiceProperty.USING_DISTANT);
				break;
			}
			limit += p;
		}
	}
}