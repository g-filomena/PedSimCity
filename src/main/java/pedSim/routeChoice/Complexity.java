package pedSim.routeChoice;

import java.util.ArrayList;
import java.util.List;

import pedSim.agents.Agent;
import pedSim.cognitiveMap.Region;
import pedSim.engine.PedSimCity;
import pedSim.utilities.StringEnum.LandmarkType;
import sim.graph.GraphUtils;
import sim.graph.NodeGraph;
import sim.util.geo.MasonGeometry;

/**
 * The Complexity class computes wayfinding easiness and legibility complexity
 * for navigation. It includes methods for calculating wayfinding easiness
 * within a space or region based on distance and landmarks.
 */
public class Complexity {

	/**
	 * Computes the wayfinding easiness within a space between two nodes on the
	 * basis of: a) the ratio between the distance between the passed nodes and a
	 * certain maximum distance; b) the legibility of the space, based on the
	 * presence of landmarks.
	 *
	 * @param node            The current node.
	 * @param destinationNode The destination node.
	 * @param agent           The agent navigating the space.
	 * @return The wayfinding easiness value.
	 */
	public double wayfindingEasiness(NodeGraph node, NodeGraph destinationNode, Agent agent) {

		final double distanceComplexity = GraphUtils.getCachedNodesDistance(node, destinationNode)
				/ Math.max(PedSimCity.roads.MBR.getHeight(), PedSimCity.roads.MBR.getWidth());

		double buildingsComplexity = 1.0;
		List<MasonGeometry> buildings = new ArrayList<>(agent.cognitiveMap.getBuildings(node, destinationNode));
		List<MasonGeometry> landmarks = new ArrayList<>(getAgentLandmarks(agent, null));
		if (!buildings.isEmpty()) {
			landmarks.retainAll(buildings);
			buildingsComplexity = buildingsComplexity(buildings, landmarks);
		}
		double wayfindingComplexity = (distanceComplexity + buildingsComplexity) / 2.0;
		double easiness = 1.0 - wayfindingComplexity;
		return easiness;
	}

	/**
	 * Computes the wayfinding easiness within a region on the basis of: a) the
	 * ratio between the distance that would be walked within the region considered
	 * and the distance between the origin and the destination; b) the legibility of
	 * the region based on the presence of landmarks.
	 *
	 * @param currentNode     The current node within the region.
	 * @param exitGateway     The gateway node at the region exit.
	 * @param originNode      The origin node of the whole trip.
	 * @param destinationNode The destination node of the whole trip.
	 * @param agentProperties The properties of the agent.
	 * @return The wayfinding easiness value within the region.
	 */
	public double wayfindingEasinessRegion(NodeGraph currentNode, NodeGraph exitGateway, NodeGraph originNode,
			NodeGraph destinationNode, Agent agentProperties) {

		double intraRegionDistance = GraphUtils.getCachedNodesDistance(currentNode, exitGateway);
		double distance = GraphUtils.getCachedNodesDistance(originNode, destinationNode);
		double distanceComplexity = intraRegionDistance / distance;
		if (distanceComplexity < 0.25)
			return 1.0;

		Region region = PedSimCity.regionsMap.get(currentNode.regionID);
		double buildingsComplexity = buildingsRegionComplexity(region, agentProperties);
		double wayfindingComplexity = (distanceComplexity + buildingsComplexity) / 2.0;
		return 1.0 - wayfindingComplexity;
	}

	/**
	 * Computes the complexity of a certain area based on the presence of landmarks.
	 *
	 * @param buildings The set of buildings in the area.
	 * @param landmarks The set of landmarks in the area.
	 * @return The complexity value of the area.
	 */
	public double buildingsComplexity(List<MasonGeometry> buildings, List<MasonGeometry> landmarks) {
		return ((double) buildings.size() - landmarks.size()) / buildings.size();
	}

	/**
	 * Computes the building-based complexity of a region based on the type of
	 * landmarkness (local or global).
	 *
	 * @param region The region for which complexity is calculated.
	 * @param agent  The agent navigating the region.
	 * @return The building-based complexity of the region.
	 */
	public double buildingsRegionComplexity(Region region, Agent agent) {

		List<MasonGeometry> landmarks;
		List<MasonGeometry> buildings = new ArrayList<>(agent.cognitiveMap.getBuildingsWithinRegion(region));
		if (buildings.isEmpty())
			return 1.0;

		LandmarkType agentLandmarkType = agent.getProperties().landmarkType;
		if (agentLandmarkType.equals(LandmarkType.LOCAL))
			landmarks = new ArrayList<>(agent.cognitiveMap.getRegionLocalLandmarks(region));
		else
			landmarks = new ArrayList<>(agent.cognitiveMap.getRegionGlobalLandmarks(region));

		return buildingsComplexity(buildings, landmarks);
	}

	/**
	 * Retrieves landmarks for the agent based at the city or region level (if
	 * provided).
	 *
	 * @param agent  The agent for which landmarks are retrieved.
	 * @param region The region for which landmarks are retrieved (can be null).
	 * @return A list of landmarks for the agent.
	 */
	public List<MasonGeometry> getAgentLandmarks(Agent agent, Region region) {

		LandmarkType agentLandmarkType = agent.getProperties().landmarkType;
		if (region != null) {
			if (agentLandmarkType.equals(LandmarkType.LOCAL))
				return agent.cognitiveMap.getRegionLocalLandmarks(region);
			else
				return agent.cognitiveMap.getRegionGlobalLandmarks(region);
		} else {
			if (agentLandmarkType.equals(LandmarkType.LOCAL))
				return agent.cognitiveMap.getLocalLandmarks().getGeometries();
			else
				return agent.cognitiveMap.getGlobalLandmarks().getGeometries();
		}
	}
}