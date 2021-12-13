package pedsimcity.agents;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.List;
import java.util.Random;

import org.javatuples.Pair;

import pedsimcity.main.UserParameters;
import pedsimcity.utilities.Utilities;

public class AgentGroupProperties extends AgentProperties {

	String groupName;
	double portion = 0.0;
	public double agentKnowledge = 0.0;
	Group group;
	public boolean usingElements = false;
	public boolean elementsActivated = false;

	ArrayList<Double> pElements = new ArrayList<>(Arrays.asList(0.0, 0.0));
	ArrayList<Double> pOnlyMinimisation = new ArrayList<>(Arrays.asList(0.0, 0.0));
	ArrayList<Double> pHeuristics = new ArrayList<>(Arrays.asList(0.0, 0.0));
	ArrayList<Double> pRegionBased = new ArrayList<>(Arrays.asList(0.0));
	ArrayList<Double> pSubGoals = new ArrayList<>(Arrays.asList(0.0, 0.0));
	ArrayList<Double> pDistantLandmarks = new ArrayList<>(Arrays.asList(0.0));

	HashMap<String, Double> pElementsMap = new HashMap<>();
	HashMap<String, Double> pOnlyMinimisationMap = new HashMap<>();
	HashMap<String, Double> pHeuristicsMap = new HashMap<>();
	HashMap<String, Double> pRegionBasedMap = new HashMap<>();
	HashMap<String, Double> pSubGoalsMap = new HashMap<>();
	HashMap<String, Double> pDistantLandmarksMap = new HashMap<>();
	HashMap<String, Double> pRandomElementsMap = new HashMap<>();

	ArrayList<String> elements = new ArrayList<>(Arrays.asList("usingElements", "notUsingElements"));
	ArrayList<String> onlyMinimisation = new ArrayList<>(Arrays.asList("roadDistance", "angularChange"));
	ArrayList<String> localHeuristics = new ArrayList<>(Arrays.asList("roadDistance", "angularChange"));
	ArrayList<String> subGoals = new ArrayList<>(Arrays.asList("localLandmarks", "barrierSubGoals", "noSubGoals"));
	ArrayList<String> regionBased = new ArrayList<>(Arrays.asList("regionBased", "notRegionBased"));
	ArrayList<String> distantLandmarks = new ArrayList<>(Arrays.asList("usingDistant", "notUsingDistant"));
	ArrayList<String> randomElements = new ArrayList<>(
			Arrays.asList("regionBased", "localLandmarks", "barrierSubGoals", "usingDistant"));

	public AgentGroupProperties() {
	}

	public AgentGroupProperties(Group group) {
		this.group = group;
		this.groupName = this.group.groupName;
	}

	public void setParametersFromGroup() {

		if (this.groupName.equals("nullGroup"))
			return;

		final Pair<Double, Double> pUsingElements = new Pair<>(this.group.pUsingElements, this.group.pUsingElementsSD);
		final Pair<Double, Double> pNotUsingElements = new Pair<>(this.group.pNotUsingElements,
				this.group.pNotUsingElementsSD);
		final ArrayList<Pair<Double, Double>> pElementsDis = new ArrayList<>(
				Arrays.asList(pUsingElements, pNotUsingElements));

		this.updateProbabilities(this.pElements, pElementsDis);
		for (int n = 0; n != this.pElements.size(); n++)
			this.pElementsMap.put(this.elements.get(n), this.pElements.get(n));

		// only minimisation
		final Pair<Double, Double> pOnlyRoadDistance = new Pair<>(this.group.pOnlyRoadDistance,
				this.group.pOnlyRoadDistanceSD);
		final Pair<Double, Double> pOnlyAngularChange = new Pair<>(this.group.pOnlyAngularChange,
				this.group.pOnlyAngularChangeSD);
		final ArrayList<Pair<Double, Double>> pOnlyMinimisationDis = new ArrayList<>(
				Arrays.asList(pOnlyRoadDistance, pOnlyAngularChange));
		this.updateProbabilities(this.pOnlyMinimisation, pOnlyMinimisationDis);
		for (int n = 0; n != this.pOnlyMinimisation.size(); n++)
			this.pOnlyMinimisationMap.put(this.onlyMinimisation.get(n), this.pOnlyMinimisation.get(n));

		// heuristics
		final Pair<Double, Double> pRoadDistance = new Pair<>(this.group.pRoadDistance, this.group.pRoadDistanceSD);
		final Pair<Double, Double> pAngularChange = new Pair<>(this.group.pAngularChange, this.group.pAngularChangeSD);
		final ArrayList<Pair<Double, Double>> pHeuristicsDis = new ArrayList<>(
				Arrays.asList(pRoadDistance, pAngularChange));
		this.updateProbabilities(this.pHeuristics, pHeuristicsDis);
		for (int n = 0; n != this.pHeuristics.size(); n++)
			this.pHeuristicsMap.put(this.localHeuristics.get(n), this.pHeuristics.get(n));

		// Coarse plan
		final Pair<Double, Double> pRegionBasedNavigation = new Pair<>(this.group.pRegionBasedNavigation,
				this.group.pRegionBasedNavigationSD);
		final ArrayList<Pair<Double, Double>> pRegionsDis = new ArrayList<>(Arrays.asList(pRegionBasedNavigation));
		this.updateProbabilities(this.pRegionBased, pRegionsDis);
		for (int n = 0; n != this.regionBased.size(); n++) {
			double p = 0.0;
			if (n == 1)
				p = 1.0 - this.pRegionBased.get(0);
			else
				p = this.pRegionBased.get(0);
			this.pRegionBasedMap.put(this.regionBased.get(n), p);
		}

		// subgoals
		final Pair<Double, Double> pLocalLandmarks = new Pair<>(this.group.pLocalLandmarks,
				this.group.pLocalLandmarksSD);
		final Pair<Double, Double> pBarrierSubGoals = new Pair<>(this.group.pBarrierSubGoals,
				this.group.pBarrierSubGoalsSD);

		final ArrayList<Pair<Double, Double>> pSubGoalsDis = new ArrayList<>(
				Arrays.asList(pLocalLandmarks, pBarrierSubGoals));
		this.updateProbabilities(this.pSubGoals, pSubGoalsDis);
		for (int n = 0; n != this.subGoals.size(); n++) {
			double p = 0.0;
			if (n < 2)
				p = this.pSubGoals.get(n);
			else
				p = 1.00 - this.pSubGoals.stream().mapToDouble(d -> d).sum();
			this.pSubGoalsMap.put(this.subGoals.get(n), p);
		}

		// distant landmarks
		final Pair<Double, Double> pDistantLandmarks = new Pair<>(this.group.pDistantLandmarks,
				this.group.pDistantLandmarksSD);
		final ArrayList<Pair<Double, Double>> pDistantLandmarksDis = new ArrayList<>(Arrays.asList(pDistantLandmarks));
		this.updateProbabilities(this.pDistantLandmarks, pDistantLandmarksDis);

		for (int n = 0; n != this.distantLandmarks.size(); n++) {
			double p = 0.0;
			if (n == 1)
				p = 1.00 - this.pDistantLandmarks.get(n - 1);
			else
				p = this.pDistantLandmarks.get(n);
			this.pDistantLandmarksMap.put(this.distantLandmarks.get(n), p);
		}

		// other elements
		this.pRandomElementsMap.put(this.randomElements.get(0), this.pRegionBased.get(0));
		this.pRandomElementsMap.put(this.randomElements.get(1), this.pSubGoals.get(0));
		this.pRandomElementsMap.put(this.randomElements.get(2), this.pSubGoals.get(1));
		this.pRandomElementsMap.put(this.randomElements.get(3), this.pDistantLandmarks.get(0));

		// other route properties

		this.naturalBarriers = this.rescale(0.0, 1.0, 1.00, 0.00, this.group.naturalBarriers);
		this.naturalBarriersSD = this.group.naturalBarriersSD;
		this.severingBarriers = this.rescale(0.0, 1.0, 1.00, 2.00, this.group.severingBarriers);
		this.severingBarriersSD = this.group.severingBarriersSD;

	}

	public void defineRouteChoiceParameters() {

		this.reset();

		if (this.groupName.equals("population"))
			if (UserParameters.shortestPathOnly) {
				this.onlyMinimising = "angularChange";
				return;
			}

		if (this.groupName.equals("nullGroup")) {
			if (UserParameters.shortestPathOnly) {
				this.onlyMinimising = "roadDistance";
				return;
			}
			this.fromUniform();
		} else
			this.setParametersFromGroup();

		final Random random = new Random();

		// using elements or not
		List<String> keys = new ArrayList<>(this.pElementsMap.keySet());
		double pRandom = random.nextDouble() * this.pElementsMap.values().stream().mapToDouble(d -> d).sum();
		double limit = 0.0;
		for (final String key : keys) {
			final double p = this.pElementsMap.get(key);
			if (pRandom <= p + limit) {
				if (key.equals("usingElements"))
					this.usingElements = true;
				break;
			}
			limit += p;
		}

		// minimisation approaches
		if (!this.usingElements) {
			keys.clear();
			keys = new ArrayList<>(this.pOnlyMinimisationMap.keySet());
			pRandom = random.nextDouble() * this.pOnlyMinimisationMap.values().stream().mapToDouble(d -> d).sum();
			limit = 0.0;
			for (final String key : keys) {
				final double p = this.pOnlyMinimisationMap.get(key);
				if (pRandom <= p + limit) {
					this.onlyMinimising = key;
					break;
				}
				limit += p;
			}
			return;
		}

		if (!UserParameters.shortestPathOnly) {
			if (this.naturalBarriers < 0.95)
				this.preferenceNaturalBarriers = true;
			if (this.severingBarriers > 1.05)
				this.aversionSeveringBarriers = true;
		}

		// local minimisation heuristic
		keys.clear();
		keys = new ArrayList<>(this.pHeuristicsMap.keySet());
		pRandom = random.nextDouble() * this.pHeuristicsMap.values().stream().mapToDouble(d -> d).sum();

		limit = 0.0;
		for (final String key : keys) {
			final double p = this.pHeuristicsMap.get(key);
			if (pRandom <= p + limit) {
				this.localHeuristic = key;
				break;
			}
			limit += p;
		}

		while (this.usingElements && !this.elementsActivated)
			this.activateElements();
	}

	public void reset() {
		this.onlyMinimising = "";
		this.localHeuristic = "";
		this.typeBarriers = "";
		this.landmarkBasedNavigation = false;
		this.barrierBasedNavigation = false;
		this.regionBasedNavigation = false;
		this.usingDistantLandmarks = false;
		this.preferenceNaturalBarriers = false;
		this.aversionSeveringBarriers = false;
		this.usingElements = false;
		this.elementsActivated = false;
	}

	public void fromUniform() {

		this.complementaryUniform(this.elements, this.pElementsMap);
		this.complementaryUniform(this.onlyMinimisation, this.pOnlyMinimisationMap);
		this.complementaryUniform(this.localHeuristics, this.pHeuristicsMap);
		this.complementaryUniform(this.regionBased, this.pRegionBasedMap);
		this.complementaryUniform(this.subGoals, this.pSubGoalsMap);
		this.complementaryUniform(this.distantLandmarks, this.pDistantLandmarksMap);

		this.naturalBarriers = 0.00 + Math.random() * (1.00 - 0.00);
		this.severingBarriers = 1.00 + Math.random() * (2.00 - 1.00);
	}

	public void updateProbabilities(ArrayList<Double> probabilities, ArrayList<Pair<Double, Double>> pDistribution) {

		for (final Double d : probabilities) {
			final int index = probabilities.indexOf(d);
			final double p = Utilities.fromDistribution(pDistribution.get(index).getValue0(),
					pDistribution.get(index).getValue1(), null);
			probabilities.set(index, p);
		}
	}

	public double rescale(double oldMin, double oldMax, double newMin, double newMax, double value) {

		final double oldRange = oldMax - oldMin;
		final double newRange = newMax - newMin;
		return (value - oldMin) * newRange / oldRange + newMin;
	}

	public void complementaryUniform(ArrayList<String> variables, HashMap<String, Double> map) {

		final double probability = 1.0 / Double.valueOf(variables.size());
		for (final String s : variables)
			map.put(s, probability);
	}

	private void activateElements() {

		final Random random = new Random();
		List<String> keys = new ArrayList<>(this.pRegionBasedMap.keySet());

		double pRandom = random.nextDouble() * this.pRegionBasedMap.values().stream().mapToDouble(d -> d).sum();
		double limit = 0.0;
		for (final String key : keys) {
			final double p = this.pRegionBasedMap.get(key);
			if (pRandom <= p + limit) {
				if (key.equals("regionBased")) {
					this.regionBasedNavigation = true;
					this.elementsActivated = true;
				}
				break;
			}
			limit += p;
		}

		// subgoals
		keys.clear();
		keys = new ArrayList<>(this.pSubGoalsMap.keySet());
		pRandom = random.nextDouble() * this.pSubGoalsMap.values().stream().mapToDouble(d -> d).sum();
		limit = 0.0;
		for (final String key : keys) {
			final double p = this.pSubGoalsMap.get(key);
			if (pRandom <= p + limit) {
				if (key.equals("localLandmarks")) {
					this.landmarkBasedNavigation = true;
					this.elementsActivated = true;
				}
				if (key.equals("barrierSubGoals")) {
					this.barrierBasedNavigation = true;
					this.elementsActivated = true;
					this.typeBarriers = "separating";
				}
				break;
			}
			limit += p;
		}

		// global landmarks
		keys.clear();
		keys = new ArrayList<>(this.pDistantLandmarksMap.keySet());
		pRandom = random.nextDouble() * this.pDistantLandmarksMap.values().stream().mapToDouble(d -> d).sum();
		limit = 0.0;
		for (final String key : keys) {
			final double p = this.pDistantLandmarksMap.get(key);
			if (pRandom <= p + limit) {
				if (key.equals("usingDistant")) {
					this.elementsActivated = true;
					this.usingDistantLandmarks = true;
				}
				break;
			}
			limit += p;
		}
	}
}
