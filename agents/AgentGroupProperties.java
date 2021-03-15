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

	ArrayList<Double> pOnlyMinimisation = new ArrayList<Double>(Arrays.asList(0.0, 0.0, 0.0));
	ArrayList<Double> pHeuristics = new ArrayList<Double>(Arrays.asList(0.0, 0.0, 0.0));
	ArrayList<Double> pSubGoals = new ArrayList<Double>(Arrays.asList(0.0, 0.0));

	HashMap<String, Double> pOnlyMinimisationMap = new HashMap<String, Double>();
	HashMap<String, Double> pHeuristicsMap = new HashMap<String, Double>();
	HashMap<String, Double> pSubGoalsMap = new HashMap<String, Double>();

	List<String> onlyMinimisation = Arrays.asList("roadDistance", "angularChange", "turns");
	List<String> localHeuristics = Arrays.asList("roadDistance", "angularChange");
	List<String> subGoals = Arrays.asList("localLandmarks", "barrierSubGoals");

	public void setPropertiesFromGroup(Group group) {

		this.groupName = group.groupName;
		Random random = new Random();

		// only minimisation
		Pair<Double, Double> pOnlyRoadDistance = new Pair<Double, Double>(group.pOnlyRoadDistance, group.pOnlyRoadDistanceSD);
		Pair<Double, Double> pOnlyAngularChange = new Pair<Double, Double>(group.pOnlyAngularChange, group.pOnlyAngularChangeSD);
		Pair<Double, Double> pOnlyTurns = new Pair<Double, Double>(group.pOnlyTurns, group.pOnlyTurnsSD);
		ArrayList<Pair<Double, Double>> pOnlyMinimisationDis = new ArrayList<Pair<Double, Double>>(Arrays.asList(pOnlyRoadDistance, pOnlyAngularChange,
				pOnlyTurns));

		int g;
		double remainder = 1.0;
		List<Integer> processed = new ArrayList<Integer>();
		while (processed.size() != pOnlyMinimisation.size()) {
			do {
				g = random.nextInt(pOnlyMinimisation.size());
			} while (processed.contains(g));

			pOnlyMinimisation.set(g, Utilities.fromDistribution(pOnlyMinimisationDis.get(g).getValue0(), pOnlyMinimisationDis.get(g).getValue1(), null));
			if (pOnlyMinimisation.get(g) > remainder) pOnlyMinimisation.set(g, remainder);
			remainder -= pOnlyMinimisation.get(g);
			processed.add(g);
			if (processed.size() == pOnlyMinimisation.size()) break;
		}

		for (int n = 0 ; n != pOnlyMinimisation.size() ; n++) pOnlyMinimisationMap.put(onlyMinimisation.get(n), pOnlyMinimisation.get(n));

		// heuristics
		Pair<Double, Double> pRoadDistance = new Pair<Double, Double>(group.pRoadDistance, group.pRoadDistanceSD);
		Pair<Double, Double> pAngularChange = new Pair<Double, Double>(group.pAngularChange, group.pAngularChangeSD);
		ArrayList<Pair<Double, Double>> pHeuristicsDis = new ArrayList<Pair<Double, Double>>(Arrays.asList(pRoadDistance, pAngularChange));

		processed.clear();
		int i;
		remainder = 1.0;
		while (processed.size() != pHeuristics.size()) {
			do {
				i = random.nextInt(pHeuristics.size());
			} while (processed.contains(i));

			pHeuristics.set(i, Utilities.fromDistribution(pHeuristicsDis.get(i).getValue0(), pHeuristicsDis.get(i).getValue1(), null));
			if (pHeuristics.get(i) > remainder) pHeuristics.set(i, remainder);
			remainder -= pHeuristics.get(i);
			processed.add(i);
			if (processed.size() == pHeuristics.size()) break;
		}

		for (int n = 0 ; n != pHeuristics.size() ; n++) pHeuristicsMap.put(localHeuristics.get(n), pHeuristics.get(n));

		// subgoals
		Pair<Double, Double> pLocalLandmarks = new Pair<Double, Double>(group.pLocalLandmarks, group.pLocalLandmarksSD);
		Pair<Double, Double> pBarrierSubGoals = new Pair<Double, Double>(group.pBarrierSubGoals, group.pBarrierSubGoalsSD);
		ArrayList<Pair<Double, Double>> pSubGoalsDis = new ArrayList<Pair<Double, Double>>(Arrays.asList(pLocalLandmarks, pBarrierSubGoals));

		processed.clear();
		int t;
		remainder = 1.0;
		while (processed.size() != pSubGoals.size()) {
			do {
				t = random.nextInt(pSubGoals.size());
			} while (processed.contains(t));

			pSubGoals.set(t, Utilities.fromDistribution(pSubGoalsDis.get(t).getValue0(), pSubGoalsDis.get(t).getValue1(), null));
			if (pSubGoals.get(t) > remainder) pSubGoals.set(t, remainder);
			remainder -= pSubGoals.get(t);
			processed.add(t);
			if (processed.size() == pSubGoals.size()) break;
		}

		for (int n = 0 ; n != pSubGoals.size() ; n++) pSubGoalsMap.put(subGoals.get(n), pSubGoals.get(n));

		// Coarse plan
		this.pRegionBasedNavigation = Utilities.fromDistribution(group.pRegionBasedNavigation, group.pRegionBasedNavigationSD, null);

		// other route properties
		this.pGlobalLandmarks = Utilities.fromDistribution(group.pGlobalLandmarks, group.pGlobalLandmarksSD, null);
		this.meanNaturalBarriers = Utilities.fromDistribution(group.meanNaturalBarriers, group.meanNaturalBarriersSD, null);
		this.meanSeveringBarriers = Utilities.fromDistribution(group.meanSeveringBarriers, group.meanSeveringBarriersSD, null);

//		this.agentKnowledge = group.agentKnowledgeMin + random.nextDouble() * (group.agentKnowledgeMax - group.agentKnowledgeMin);

	}

	public void defineRouteChoiceParameters() {

		this.reset();
		Random random = new Random();
		double phRandom = random.nextDouble();
		double limit = 0.0;

		List<String> keys = new ArrayList<String>(pOnlyMinimisationMap.keySet());
		Collections.shuffle(keys);
		for (String key : keys)  {
			double pHeuristic = pOnlyMinimisationMap.get(key);
			double pValue = pHeuristic + limit;
			if (phRandom <= pValue) {
				this.onlyMinimising = key;
				break;
			}
			limit = pValue;
		}

		if (this.onlyMinimising != null) return;

		keys = new ArrayList<String>(pHeuristicsMap.keySet());
		Collections.shuffle(keys);
		for (String key : keys)  {
			double pHeuristic = pHeuristicsMap.get(key);
			double pValue = pHeuristic + limit;
			if (phRandom <= pValue) {
				this.localHeuristic = key;
				break;
			}
			limit = pValue;
		}

		keys.clear();
		keys = new ArrayList<String>(pSubGoalsMap.keySet());
		Collections.shuffle(keys);
		double psRandom = random.nextDouble();
		limit = 0.0;
		for (String key : keys)  {
			double pHeuristic = pSubGoalsMap.get(key);
			double pValue = pHeuristic + limit;
			if (psRandom <= pValue) {
				if (key.equals("localLandmarks")) this.landmarkBasedNavigation = true;
				if (key.equals("barrierSubGoals")) this.barrierBasedNavigation = true;
				break;
			}
			limit = pHeuristic;
		}

		// other route properties
		double rbRandom = random.nextDouble();
		if (rbRandom <= this.pRegionBasedNavigation) this.regionBasedNavigation = true;
		double glRandom = random.nextDouble();
		if (glRandom <= this.pGlobalLandmarks) this.usingGlobalLandmarks = true;

		if (meanNaturalBarriers != 1.00) this.preferenceNaturalBarriers = true;
		if (meanSeveringBarriers != 1.00) this.aversionSeveringBarriers = true;

	}

	public void reset() {
		this.onlyMinimisation = null;
		this.localHeuristics = null;
		this.landmarkBasedNavigation = false;
		this.barrierBasedNavigation = false;
		this.regionBasedNavigation = false;
		this.usingGlobalLandmarks = false;
	}




}
