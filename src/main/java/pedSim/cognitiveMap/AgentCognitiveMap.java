package pedSim.cognitiveMap;

import sim.field.geo.VectorLayer;

/**
 * Represents an agent's cognitive map, which provides access to various map
 * attributes. In this version of PedSimCity, this is a simple structure
 * designed for further developments.
 */
public class AgentCognitiveMap extends CommunityCognitiveMap {

	/**
	 * Constructs an AgentCognitiveMap.
	 */
	public AgentCognitiveMap() {
	}

	/**
	 * Gets the local landmarks from the cognitive map.
	 *
	 * @return The local landmarks.
	 */
	public VectorLayer getLocalLandmarks() {
		return CommunityCognitiveMap.localLandmarks;
	}

	/**
	 * Gets the global landmarks from the cognitive map.
	 *
	 * @return The global landmarks.
	 */
	public VectorLayer getGlobalLandmarks() {
		return CommunityCognitiveMap.globalLandmarks;
	}

	/**
	 * Gets the barriers from the cognitive map.
	 *
	 * @return The barriers.
	 */
	public VectorLayer getBarriers() {
		return CommunityCognitiveMap.barriers;
	}

	/**
	 * Verifies if any barriers are represented (contained) in the cognitive map.
	 *
	 * @return The barriers.
	 */
	public boolean containsBarriers() {
		return !getBarriers().getGeometries().isEmpty();
	}
}
