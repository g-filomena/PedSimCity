package pedsim.empirical.agent;

import java.util.Arrays;
import java.util.EnumMap;
import java.util.List;
import java.util.Map;
import java.util.Random;

import pedsim.core.agents.Agent;
import pedsim.core.agents.AgentProperties;
import pedsim.core.utilities.StringEnum.AgentBarrierType;
import pedsim.core.utilities.StringEnum.LandmarkType;
import pedsim.core.utilities.StringEnum.LocalHeuristicMode;
import pedsim.core.utilities.StringEnum.MinimisationMode;
import pedsim.core.utilities.StringEnum.RouteChoiceProperty;

/**
 * Agent properties sampled from empirical group-level parameters.
 *
 * This class adapts the legacy empirical-group logic to the current
 * core.AgentProperties API, which is setter/enum based rather than public-field
 * based.
 */
public class EmpiricalAgentProperties extends AgentProperties {

	/*
	 * IMPORTANT: this range intentionally follows the empirical model released in
	 * PedSimCity v1.11, where natural barriers are rescaled as 1.00 -> 0.00.
	 *
	 * Natural barriers are not treated like severing obstacles here. A higher
	 * empirical tendency towards natural barriers must produce a lower routing-cost
	 * factor, i.e. a preference / perceived-cost discount.
	 *
	 * The constant names are legacy semantic names, not numeric ordering: -
	 * MIN_NATURAL_BARRIERS = 1.00 is passed as the output value for the low end of
	 * the empirical probability range. - MAX_NATURAL_BARRIERS = 0.00 is passed as
	 * the output value for the high end of the empirical probability range.
	 *
	 * Therefore the correct call is:
	 *
	 * rescale(0.0, 1.0, MIN_NATURAL_BARRIERS, MAX_NATURAL_BARRIERS, value)
	 *
	 * This intentionally maps: 0.0 -> 1.0 1.0 -> 0.0
	 *
	 * Do not reorder these arguments to MAX_NATURAL_BARRIERS, MIN_NATURAL_BARRIERS:
	 * that would remove the inversion and change the empirical model semantics.
	 *
	 * Severing barriers are different: they follow the normal direction, where a
	 * higher empirical tendency means a higher perceived-cost factor.
	 */
	private static final double MIN_NATURAL_BARRIERS = 1.00;
	private static final double MAX_NATURAL_BARRIERS = 0.00;
	private static final double MIN_SEVERING_BARRIERS = 1.00;
	private static final double MAX_SEVERING_BARRIERS = 2.00;

	private final Random random = new Random();

	public final EmpiricalGroup groupName;
	private final EmpiricalAgentsGroup group;

	private final List<RouteChoiceProperty> elements = Arrays.asList(RouteChoiceProperty.USING_ELEMENTS,
			RouteChoiceProperty.NOT_USING_ELEMENTS);

	private final List<RouteChoiceProperty> minimisation = Arrays.asList(RouteChoiceProperty.ROAD_DISTANCE,
			RouteChoiceProperty.ANGULAR_CHANGE);

	private final List<RouteChoiceProperty> localHeuristics = Arrays.asList(RouteChoiceProperty.ROAD_DISTANCE_LOCAL,
			RouteChoiceProperty.ANGULAR_CHANGE_LOCAL);

	private final List<RouteChoiceProperty> regionBased = Arrays.asList(RouteChoiceProperty.REGION_BASED,
			RouteChoiceProperty.NOT_REGION_BASED);

	private final List<RouteChoiceProperty> subGoals = Arrays.asList(RouteChoiceProperty.LOCAL_LANDMARKS,
			RouteChoiceProperty.BARRIER_SUBGOALS, RouteChoiceProperty.NO_SUBGOALS);

	private final List<RouteChoiceProperty> distantLandmarks = Arrays.asList(RouteChoiceProperty.USING_DISTANT,
			RouteChoiceProperty.NOT_USING_DISTANT);

	private final Map<RouteChoiceProperty, Double> elementsMap = new EnumMap<>(RouteChoiceProperty.class);

	private final Map<RouteChoiceProperty, Double> minimisationMap = new EnumMap<>(RouteChoiceProperty.class);

	private final Map<RouteChoiceProperty, Double> localHeuristicsMap = new EnumMap<>(RouteChoiceProperty.class);

	private final Map<RouteChoiceProperty, Double> regionBasedMap = new EnumMap<>(RouteChoiceProperty.class);

	private final Map<RouteChoiceProperty, Double> subGoalsMap = new EnumMap<>(RouteChoiceProperty.class);

	private final Map<RouteChoiceProperty, Double> distantLandmarksMap = new EnumMap<>(RouteChoiceProperty.class);

	public EmpiricalAgentProperties(Agent agent, EmpiricalAgentsGroup group) {
		super();
		this.group = group;
		this.groupName = group.groupName;
	}

	public void randomizeRouteChoiceParameters() {
		reset();

		if (groupName == EmpiricalGroup.NULLGROUP) {
			initialiseUniformProbabilities();
		} else {
			setParametersFromGroup();
		}

		boolean usingElements = weightedChoice(elementsMap,
				RouteChoiceProperty.NOT_USING_ELEMENTS) == RouteChoiceProperty.USING_ELEMENTS;

		if (!usingElements) {
			applyPureMinimisation();
			return;
		}

		applyBarrierPreferences();
		applyLocalHeuristic();
		activateElements();
	}

	private void setParametersFromGroup() {
		elementsMap.put(RouteChoiceProperty.USING_ELEMENTS,
				sampleProbability(group.probabilityUsingElements, group.probabilityUsingElementsSD));
		elementsMap.put(RouteChoiceProperty.NOT_USING_ELEMENTS,
				sampleProbability(group.probabilityNotUsingElements, group.probabilityNotUsingElementsSD));

		minimisationMap.put(RouteChoiceProperty.ROAD_DISTANCE,
				sampleProbability(group.probabilityRoadDistance, group.probabilityRoadDistanceSD));
		minimisationMap.put(RouteChoiceProperty.ANGULAR_CHANGE,
				sampleProbability(group.probabilityAngularChange, group.probabilityAngularChangeSD));

		localHeuristicsMap.put(RouteChoiceProperty.ROAD_DISTANCE_LOCAL,
				sampleProbability(group.probabilityLocalRoadDistance, group.probabilityLocalRoadDistanceSD));
		localHeuristicsMap.put(RouteChoiceProperty.ANGULAR_CHANGE_LOCAL,
				sampleProbability(group.probabilityLocalAngularChange, group.probabilityLocalAngularChangeSD));

		double pRegion = sampleProbability(group.probabilityRegionBasedNavigation,
				group.probabilityRegionBasedNavigationSD);
		regionBasedMap.put(RouteChoiceProperty.REGION_BASED, pRegion);
		regionBasedMap.put(RouteChoiceProperty.NOT_REGION_BASED, 1.0 - pRegion);

		double pLocalLandmarks = sampleProbability(group.probabilityLocalLandmarks, group.probabilityLocalLandmarksSD);
		double pBarrierSubGoals = sampleProbability(group.probabilityBarrierSubGoals,
				group.probabilityBarrierSubGoalsSD);
		double pNoSubGoals = Math.max(0.0, 1.0 - pLocalLandmarks - pBarrierSubGoals);

		subGoalsMap.put(RouteChoiceProperty.LOCAL_LANDMARKS, pLocalLandmarks);
		subGoalsMap.put(RouteChoiceProperty.BARRIER_SUBGOALS, pBarrierSubGoals);
		subGoalsMap.put(RouteChoiceProperty.NO_SUBGOALS, pNoSubGoals);

		double pDistant = sampleProbability(group.probabilityDistantLandmarks, group.probabilityDistantLandmarksSD);
		distantLandmarksMap.put(RouteChoiceProperty.USING_DISTANT, pDistant);
		distantLandmarksMap.put(RouteChoiceProperty.NOT_USING_DISTANT, 1.0 - pDistant);

		// TODO verify naturalBarrier is correctly inverted as a preference rather than
		// a cost factor
		double naturalBarrierScore = rescale(0.0, 1.0, MIN_NATURAL_BARRIERS, MAX_NATURAL_BARRIERS,
				sampleProbability(group.naturalBarriers, group.naturalBarriersSD));
		double severingBarrierScore = rescale(0.0, 1.0, MIN_SEVERING_BARRIERS, MAX_SEVERING_BARRIERS,
				sampleProbability(group.severingBarriers, group.severingBarriersSD));

		setNaturalBarriersMean(naturalBarrierScore);
		setNaturalBarriersSD(group.naturalBarriersSD);
		setSeveringBarriersMean(severingBarrierScore);
		setSeveringBarriersSD(group.severingBarriersSD);
	}

	private void initialiseUniformProbabilities() {
		initialiseUniform(elements, elementsMap);
		initialiseUniform(minimisation, minimisationMap);
		initialiseUniform(localHeuristics, localHeuristicsMap);
		initialiseUniform(regionBased, regionBasedMap);
		initialiseUniform(subGoals, subGoalsMap);
		initialiseUniform(distantLandmarks, distantLandmarksMap);

		setNaturalBarriersMean(random.nextDouble());
		setSeveringBarriersMean(1.0 + random.nextDouble());
	}

	private void applyPureMinimisation() {
		RouteChoiceProperty choice = weightedChoice(minimisationMap, RouteChoiceProperty.ROAD_DISTANCE);

		if (choice == RouteChoiceProperty.ROAD_DISTANCE) {
			setMinimisationMode(MinimisationMode.DISTANCE);
		} else {
			setMinimisationMode(MinimisationMode.ANGULAR);
		}
	}

	private void applyBarrierPreferences() {
		if (getNaturalBarriersMean() < 0.95) {
			setPreferenceNaturalBarriers(true);
		}

		if (getSeveringBarriersMean() > 1.05) {
			setAversionSeveringBarriers(true);
		}
	}

	private void applyLocalHeuristic() {
		RouteChoiceProperty choice = weightedChoice(localHeuristicsMap, RouteChoiceProperty.ROAD_DISTANCE_LOCAL);

		if (choice == RouteChoiceProperty.ROAD_DISTANCE_LOCAL) {
			setLocalHeuristicMode(LocalHeuristicMode.DISTANCE);
		} else {
			setLocalHeuristicMode(LocalHeuristicMode.ANGULAR);
		}
	}

	private void activateElements() {
		RouteChoiceProperty regionChoice = weightedChoice(regionBasedMap, RouteChoiceProperty.NOT_REGION_BASED);
		setRegionBasedNavigation(regionChoice == RouteChoiceProperty.REGION_BASED);

		RouteChoiceProperty subGoalChoice = weightedChoice(subGoalsMap, RouteChoiceProperty.NO_SUBGOALS);

		if (subGoalChoice == RouteChoiceProperty.LOCAL_LANDMARKS) {
			setUsingLocalLandmarks(true);
			setLandmarkType(LandmarkType.LOCAL);
		} else if (subGoalChoice == RouteChoiceProperty.BARRIER_SUBGOALS) {
			setBarrierBasedNavigation(true);
			setBarrierType(AgentBarrierType.SEPARATING);
			setLandmarkType(LandmarkType.LOCAL);
		}

		RouteChoiceProperty distantChoice = weightedChoice(distantLandmarksMap, RouteChoiceProperty.NOT_USING_DISTANT);
		setUsingDistantLandmarks(distantChoice == RouteChoiceProperty.USING_DISTANT);

		// Defensive fallback: if the sampled "using elements" branch activated nothing,
		// force a
		// simple local-landmark behaviour rather than leaving an empty element set.
		if (!isRegionBasedNavigation() && !isUsingLocalLandmarks() && !isBarrierBasedNavigation()
				&& !isUsingDistantLandmarks()) {
			setUsingLocalLandmarks(true);
			setLandmarkType(LandmarkType.LOCAL);
		}
	}

	private double sampleProbability(double mean, double standardDeviation) {
		if (standardDeviation <= 0.0) {
			return clamp01(mean);
		}

		double sampled = mean + random.nextGaussian() * standardDeviation;
		return clamp01(sampled);
	}

	private static double clamp01(double value) {
		return Math.max(0.0, Math.min(1.0, value));
	}

	private static void initialiseUniform(List<RouteChoiceProperty> properties,
			Map<RouteChoiceProperty, Double> target) {
		double probability = 1.0 / properties.size();
		for (RouteChoiceProperty property : properties) {
			target.put(property, probability);
		}
	}

	private RouteChoiceProperty weightedChoice(Map<RouteChoiceProperty, Double> weights, RouteChoiceProperty fallback) {
		if (weights == null || weights.isEmpty()) {
			return fallback;
		}

		double total = 0.0;
		for (double value : weights.values()) {
			total += Math.max(0.0, value);
		}

		if (total <= 0.0) {
			return fallback;
		}

		double threshold = random.nextDouble() * total;
		double cumulative = 0.0;

		for (Map.Entry<RouteChoiceProperty, Double> entry : weights.entrySet()) {
			cumulative += Math.max(0.0, entry.getValue());
			if (threshold <= cumulative) {
				return entry.getKey();
			}
		}

		return fallback;
	}

	private static double rescale(double oldMin, double oldMax, double newMin, double newMax, double value) {
		double oldRange = oldMax - oldMin;
		double newRange = newMax - newMin;
		return (value - oldMin) * newRange / oldRange + newMin;
	}
}