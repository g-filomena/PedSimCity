package pedsim.core.agents;

import java.util.Objects;
import java.util.Random;

import pedsim.core.parameters.RouteChoicePars;
import pedsim.core.utilities.StringEnum.LocalHeuristicMode;
import pedsim.core.utilities.StringEnum.MinimisationMode;
import pedsim.core.utilities.StringEnum.RouteChoiceElement;
import sim.graph.NodeGraph;

public final class Heuristics {

	private double probabilityDistanceMinimisation;
	private double probabilityAngularMinimisation;
	private double probabilityDistantLandmarks;
	private double probabilityUsingRegions;
	private double probabilityBarrierSubGoals;

	private final Agent agent;
	private final AgentProperties ap;
	private final Random random;

	private final double globalLandmarknessWeightDistance = RouteChoicePars.globalLandmarknessWeightDistanceCommunity;
	private final double globalLandmarknessWeightAngular = RouteChoicePars.globalLandmarknessWeightAngularCommunity;

	public Heuristics(Agent agent) {
		this.agent = Objects.requireNonNull(agent);
		this.ap = Objects.requireNonNull(agent.getProperties());
		this.random = new Random();
	}

	public void defineHeuristic(NodeGraph originNode, NodeGraph destinationNode, boolean onlyDistanceMinimsation) {
		if (onlyDistanceMinimsation) {
			ap.reset();
			ap.setMinimisationMode(MinimisationMode.DISTANCE);
			return;
		}
		defineRouteChoiceMechanisms();
	}

	public void defineRouteChoiceMechanisms() {

		if (isGlobalMinimisationDominant()) {
			ap.setMinimisationMode(sampleMinimisationMode());
			return;
		}

		ap.setLocalHeuristicMode(sampleLocalHeuristicMode());

		if (random.nextDouble() < probabilityBarrierSubGoals) {
			ap.addElement(RouteChoiceElement.BARRIER_BASED_NAVIGATION);
		} else {
			ap.addElement(RouteChoiceElement.LOCAL_LANDMARKS);
		}

		if (random.nextDouble() < probabilityDistantLandmarks) {
			ap.addElement(RouteChoiceElement.DISTANT_LANDMARKS);
		}

		if (random.nextDouble() < probabilityUsingRegions) {
			ap.addElement(RouteChoiceElement.REGION_BASED_NAVIGATION);
		}
	}

	private boolean isGlobalMinimisationDominant() {
		return probabilityDistanceMinimisation > 0.90 || probabilityAngularMinimisation > 0.90;
	}

	private MinimisationMode sampleMinimisationMode() {
		return sampleWeightedMode(probabilityDistanceMinimisation,
				probabilityAngularMinimisation) == BinaryMode.DISTANCE ? MinimisationMode.DISTANCE
						: MinimisationMode.ANGULAR;
	}

	private LocalHeuristicMode sampleLocalHeuristicMode() {
		return sampleWeightedMode(probabilityDistanceMinimisation,
				probabilityAngularMinimisation) == BinaryMode.DISTANCE ? LocalHeuristicMode.DISTANCE
						: LocalHeuristicMode.ANGULAR;
	}

	private BinaryMode sampleWeightedMode(double distanceWeight, double angularWeight) {
		double d = Math.max(0.0, distanceWeight);
		double a = Math.max(0.0, angularWeight);
		double total = d + a;

		if (total == 0.0) {
			return BinaryMode.DISTANCE;
		}

		return random.nextDouble() < (d / total) ? BinaryMode.DISTANCE : BinaryMode.ANGULAR;
	}

	private enum BinaryMode {
		DISTANCE, ANGULAR
	}

	public void setActivationProbabilities(double probabilityDistanceMinimisation,
			double probabilityAngularMinimisation, double probabilityDistantLandmarks, double probabilityUsingRegions,
			double probabilityBarrierSubGoals) {

		this.probabilityDistanceMinimisation = probabilityDistanceMinimisation;
		this.probabilityAngularMinimisation = probabilityAngularMinimisation;
		this.probabilityDistantLandmarks = probabilityDistantLandmarks;
		this.probabilityUsingRegions = probabilityUsingRegions;
		this.probabilityBarrierSubGoals = probabilityBarrierSubGoals;
	}

	public double getLocalLandmarksThreshold() {
		return RouteChoicePars.localLandmarkThresholdCommunity;
	}

	public double getGlobalLandmarkWeight(boolean angular) {
		return angular ? globalLandmarknessWeightAngular : globalLandmarknessWeightDistance;
	}

	public Agent getAgent() {
		return agent;
	}

	public AgentProperties getAgentProperties() {
		return ap;
	}
}