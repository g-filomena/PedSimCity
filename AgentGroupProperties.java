package sim.app.geo.PedSimCity;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.HashMap;
import java.util.List;
import java.util.Random;

import org.javatuples.Pair;

public class AgentGroupProperties extends AgentProperties {

	String groupName;
	int groupID;
	double portion = 0.0;
	double agentKnowledge = 0.0;
	ArrayList<Double> pHeuristics = new ArrayList<Double>(Arrays.asList(0.0, 0.0, 0.0));
	ArrayList<Double> pSubGoals = new ArrayList<Double>(Arrays.asList( 0.0, 0.0, 0.0));
	HashMap<String, Double> pHeuristicsMap = new HashMap<String, Double>();
	HashMap<String, Double> pSubGoalsMap = new HashMap<String, Double>();

	double pRegionBasedNavigation = 0.0;

	double pGlobalLandmarks = 0.0;
	double pNaturalBarriers = 0.0;
	double pSeveringBarriers = 0.0;
	List<String> localHeuristics = Arrays.asList("roadDistance", "angularChange", "turns");
	List<String> subGoals = Arrays.asList("localLandmarks", "barrierSubGoals", "nodeMarks");

	public void setPropertiesFromGroup(Group group) {

		this.groupName = group.groupName;
		Random random = new Random();
		// heuristics

		Pair<Double, Double> pRoadDistance = new Pair<Double, Double>(group.pRoadDistanceMin, group.pRoadDistanceMax);
		Pair<Double, Double> pAngularChange = new Pair<Double, Double>(group.pAngularChangeMin, group.pAngularChangeMax);
		Pair<Double, Double> pTurns = new Pair<Double, Double>(group.pTurnsMin, group.pTurnsMax);
		ArrayList<Pair<Double, Double>> pHeuristicsMinMax = new ArrayList<Pair<Double, Double>>(Arrays.asList(pRoadDistance, pAngularChange, pTurns));

		Pair<Double, Double> pLocalLandmarks = new Pair<Double, Double>(group.pLocalLandmarksMin, group.pLocalLandmarksMax);
		Pair<Double, Double> pBarrierSubGoals = new Pair<Double, Double>(group.pBarrierSubGoalsMin, group.pBarrierSubGoalsMax);
		Pair<Double, Double> pNodeMarks = new Pair<Double, Double>(group.pNodeMarksMin, group.pNodeMarksMax);
		ArrayList<Pair<Double, Double>> pSubGoalsMinMax = new ArrayList<Pair<Double, Double>>(Arrays.asList(pLocalLandmarks, pBarrierSubGoals, pNodeMarks));

		int i;
		double remainder = 1.0;
		List<Integer> processed = new ArrayList<Integer>();

		while (processed.size() != pHeuristics.size()) {
			do {
				i = random.nextInt(pHeuristics.size());
			} while (processed.contains(i));
			pHeuristics.set(i, pHeuristicsMinMax.get(i).getValue0() + random.nextDouble() * (pHeuristicsMinMax.get(i).getValue1() -
					pHeuristicsMinMax.get(i).getValue0()));
			if (pHeuristics.get(i) > remainder) pHeuristics.set(i, remainder);
			remainder -= pHeuristics.get(i);
			processed.add(i);
			if (processed.size() == pHeuristics.size()) break;
		}

		for (int n = 0 ; n != pHeuristics.size() ; n++) pHeuristicsMap.put(localHeuristics.get(n), pHeuristics.get(n));


		processed.clear();
		int t;
		remainder = 1.0;
		while (processed.size() != pSubGoals.size()) {
			do {
				t = random.nextInt(pSubGoals.size());
			} while (processed.contains(t));

			pSubGoals.set(t, pSubGoalsMinMax.get(t).getValue0() + random.nextDouble() * (pSubGoalsMinMax.get(t).getValue1() - pSubGoalsMinMax.get(t).getValue0()));
			if (pSubGoals.get(t) > remainder) pSubGoals.set(t, remainder);
			remainder -= pSubGoals.get(t);
			processed.add(t);
			if (processed.size() == pSubGoals.size()) break;
		}

		for (int n = 0 ; n != pSubGoals.size() ; n++) pSubGoalsMap.put(subGoals.get(n), pSubGoals.get(n));

		// Regions
		this.pRegionBasedNavigation = group.pRegionBasedNavigationMin + random.nextDouble() * (group.pRegionBasedNavigationMax - group.pRegionBasedNavigationMin);

		// other route properties
		this.pGlobalLandmarks = group.pGlobalLandmarksMin + random.nextDouble() * (group.pGlobalLandmarksMax - group.pGlobalLandmarksMin);
		this.pNaturalBarriers = group.pNaturalBarriersMin + random.nextDouble() * (group.pNaturalBarriersMax - group.pNaturalBarriersMin);
		this.pSeveringBarriers = group.pSeveringBarriersMin + random.nextDouble() * (group.pSeveringBarriersMax - group.pSeveringBarriersMin);

		this.agentKnowledge = group.agentKnowledgeMin + random.nextDouble() * (group.agentKnowledgeMax - group.agentKnowledgeMin);

	}

	public void defineRouteChoiceParameters() {

		Random random = new Random();
		// heuristics

		double phRandom = random.nextDouble();
		double limit = 0.0;

		List<String> keys = new ArrayList<String>(pHeuristicsMap.keySet());
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
				if (key.equals("nodeMarks")) this.nodeBasedNavigation = true;
				break;
			}
			limit = pHeuristic;
		}

		// other route properties
		double rbRandom = random.nextDouble();
		if (rbRandom <= this.pRegionBasedNavigation) this.regionBasedNavigation = true;
		double glRandom = random.nextDouble();
		if (glRandom <= this.pGlobalLandmarks) this.usingGlobalLandmarks = true;
		double nbRandom = random.nextDouble();
		if (nbRandom <= this.pNaturalBarriers) this.usingNaturalBarriers = true;
		double sbRandom = random.nextDouble();
		if (sbRandom <= this.pSeveringBarriers) this.avoidingSeveringBarriers = true;
	}





}
