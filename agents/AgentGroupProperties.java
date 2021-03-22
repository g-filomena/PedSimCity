package pedsimcity.agents;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.HashMap;
import java.util.List;
import java.util.Random;

import org.javatuples.Pair;

import urbanmason.main.Utilities;

public class AgentGroupProperties extends AgentProperties {

	String groupName;
	int groupID;
	double portion = 0.0;
	double agentKnowledge = 0.0;
	double pRegionBasedNavigation = 0.0;
	double pGlobalLandmarks = 0.0;

	ArrayList<Double> pOnlyMinimisation = new ArrayList<>(Arrays.asList(0.0, 0.0, 0.0));
	ArrayList<Double> pHeuristics = new ArrayList<>(Arrays.asList(0.0, 0.0, 0.0));
	ArrayList<Double> pSubGoals = new ArrayList<>(Arrays.asList(0.0, 0.0));

	HashMap<String, Double> pOnlyMinimisationMap = new HashMap<>();
	HashMap<String, Double> pHeuristicsMap = new HashMap<>();
	HashMap<String, Double> pSubGoalsMap = new HashMap<>();

	List<String> onlyMinimisation = Arrays.asList("roadDistance", "angularChange", "turns");
	List<String> localHeuristics = Arrays.asList("roadDistance", "angularChange");
	List<String> subGoals = Arrays.asList("localLandmarks", "barrierSubGoals");

	public void setPropertiesFromGroup(Group group) {

		this.groupName = group.groupName;
		if (this.groupName.equals("null"))
			return;

		// only minimisation
		final Pair<Double, Double> pOnlyRoadDistance = new Pair<>(group.pOnlyRoadDistance, group.pOnlyRoadDistanceSD);
		final Pair<Double, Double> pOnlyAngularChange = new Pair<>(group.pOnlyAngularChange,
				group.pOnlyAngularChangeSD);
		final ArrayList<Pair<Double, Double>> pOnlyMinimisationDis = new ArrayList<>(
				Arrays.asList(pOnlyRoadDistance, pOnlyAngularChange));

		this.pOnlyMinimisation = this.setProbabilities(this.pOnlyMinimisation, pOnlyMinimisationDis);
		for (int n = 0; n != this.pOnlyMinimisation.size(); n++)
			this.pOnlyMinimisationMap.put(this.onlyMinimisation.get(n), this.pOnlyMinimisation.get(n));

		// heuristics
		final Pair<Double, Double> pRoadDistance = new Pair<>(group.pRoadDistance, group.pRoadDistanceSD);
		final Pair<Double, Double> pAngularChange = new Pair<>(group.pAngularChange, group.pAngularChangeSD);
		final ArrayList<Pair<Double, Double>> pHeuristicsDis = new ArrayList<>(
				Arrays.asList(pRoadDistance, pAngularChange));
		this.pHeuristics = this.setProbabilities(this.pHeuristics, pHeuristicsDis);

		for (int n = 0; n != this.pHeuristics.size(); n++)
			this.pHeuristicsMap.put(this.localHeuristics.get(n), this.pHeuristics.get(n));

		// subgoals
		final Pair<Double, Double> pLocalLandmarks = new Pair<>(group.pLocalLandmarks, group.pLocalLandmarksSD);
		final Pair<Double, Double> pBarrierSubGoals = new Pair<>(group.pBarrierSubGoals, group.pBarrierSubGoalsSD);
		final ArrayList<Pair<Double, Double>> pSubGoalsDis = new ArrayList<>(
				Arrays.asList(pLocalLandmarks, pBarrierSubGoals));

		this.pSubGoals = this.setProbabilities(this.pSubGoals, pSubGoalsDis);

		for (int n = 0; n != this.pSubGoals.size(); n++)
			this.pSubGoalsMap.put(this.subGoals.get(n), this.pSubGoals.get(n));

		// Coarse plan
		this.pRegionBasedNavigation = Utilities.fromDistribution(group.pRegionBasedNavigation,
				group.pRegionBasedNavigationSD, null);

		// other route properties
		this.pGlobalLandmarks = Utilities.fromDistribution(group.pGlobalLandmarks, group.pGlobalLandmarksSD, null);
		this.meanNaturalBarriers = Utilities.fromDistribution(group.meanNaturalBarriers, group.meanNaturalBarriersSD,
				null);
		this.meanSeveringBarriers = Utilities.fromDistribution(group.meanSeveringBarriers, group.meanSeveringBarriersSD,
				null);
	}

	public void defineRouteChoiceParameters() {

		this.reset();
		final Random random = new Random();
		final double phRandom = random.nextDouble();
		if (this.groupName.equals("null"))
			this.fromUniform();

		double limit = 0.0;

		List<String> keys = new ArrayList<>(this.pOnlyMinimisationMap.keySet());
		Collections.shuffle(keys);
		for (final String key : keys) {
			final double pHeuristic = this.pOnlyMinimisationMap.get(key);
			final double pValue = pHeuristic + limit;
			if (phRandom <= pValue) {
				this.onlyMinimising = key;
				break;
			}
			limit = pValue;
		}

		if (this.onlyMinimising != null)
			return;

		keys = new ArrayList<>(this.pHeuristicsMap.keySet());
		Collections.shuffle(keys);
		for (final String key : keys) {
			final double pHeuristic = this.pHeuristicsMap.get(key);
			final double pValue = pHeuristic + limit;
			if (phRandom <= pValue) {
				this.localHeuristic = key;
				break;
			}
			limit = pValue;
		}

		keys.clear();
		keys = new ArrayList<>(this.pSubGoalsMap.keySet());
		Collections.shuffle(keys);
		final double psRandom = random.nextDouble();
		limit = 0.0;
		for (final String key : keys) {
			final double pHeuristic = this.pSubGoalsMap.get(key);
			final double pValue = pHeuristic + limit;
			if (psRandom <= pValue) {
				if (key.equals("localLandmarks"))
					this.landmarkBasedNavigation = true;
				if (key.equals("barrierSubGoals"))
					this.barrierBasedNavigation = true;
				break;
			}
			limit = pHeuristic;
		}

		// other route properties
		final double rbRandom = random.nextDouble();
		if (rbRandom <= this.pRegionBasedNavigation)
			this.regionBasedNavigation = true;
		final double glRandom = random.nextDouble();
		if (glRandom <= this.pGlobalLandmarks)
			this.usingGlobalLandmarks = true;

		if (this.meanNaturalBarriers != 1.00)
			this.preferenceNaturalBarriers = true;
		if (this.meanSeveringBarriers != 1.00)
			this.aversionSeveringBarriers = true;
	}

	public void reset() {
		this.onlyMinimisation = null;
		this.localHeuristics = null;
		this.landmarkBasedNavigation = false;
		this.barrierBasedNavigation = false;
		this.regionBasedNavigation = false;
		this.usingGlobalLandmarks = false;
	}

	public void fromUniform() {

		final Random udRandom = new Random();

		double remainder = 1.0;
		for (final String s : this.onlyMinimisation) {
			this.pOnlyMinimisationMap.put(s, udRandom.nextDouble());
			if (this.pOnlyMinimisationMap.get(s) > remainder)
				this.pOnlyMinimisationMap.replace(s, remainder);
			remainder -= this.pOnlyMinimisationMap.get(s);
			if (remainder < 0.0)
				remainder = 0.0;
		}

		remainder = 1.0;
		for (final String s : this.localHeuristics) {
			this.pHeuristicsMap.put(s, udRandom.nextDouble());
			if (this.pHeuristicsMap.get(s) > remainder)
				this.pHeuristicsMap.replace(s, remainder);
			remainder -= this.pHeuristicsMap.get(s);
			if (remainder < 0.0)
				remainder = 0.0;
		}

		remainder = 1.0;
		for (final String s : this.subGoals) {
			this.pSubGoalsMap.put(s, udRandom.nextDouble());
			if (this.pSubGoalsMap.get(s) > remainder)
				this.pSubGoalsMap.replace(s, remainder);
			remainder -= this.pSubGoalsMap.get(s);
			if (remainder < 0.0)
				remainder = 0.0;
		}

		// Coarse plan
		this.pRegionBasedNavigation = udRandom.nextDouble();

		// other route properties
		this.pGlobalLandmarks = udRandom.nextDouble();
		this.meanNaturalBarriers = 0.40 + Math.random() * (1.00 - 0.40);
		this.meanSeveringBarriers = 1.00 + Math.random() * (1.60 - 1.00);
	}

	public ArrayList<Double> setProbabilities(ArrayList<Double> probabilities,
			ArrayList<Pair<Double, Double>> pDistribution) {

		int t;
		double remainder = 1.0;
		final List<Integer> processed = new ArrayList<>();
		final Random random = new Random();

		remainder = 1.0;
		while (processed.size() != probabilities.size()) {
			do
				t = random.nextInt(probabilities.size());
			while (processed.contains(t));

			final double p = Utilities.fromDistribution(pDistribution.get(t).getValue0(),
					pDistribution.get(t).getValue1(), null);
			probabilities.set(t, p);
			if (p > remainder)
				probabilities.set(t, remainder);
			remainder -= probabilities.get(t);
			if (remainder < 0.0)
				remainder = 0.0;
			processed.add(t);
			if (processed.size() == probabilities.size())
				break;
		}
		return probabilities;
	}

}
