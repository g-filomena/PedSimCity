package pedsimcity.utilities;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import pedsimcity.graph.Graph;
import pedsimcity.graph.NodeGraph;
import pedsimcity.main.UserParameters;
import sim.util.Bag;
import sim.util.geo.MasonGeometry;

/**
 * A class containing functions to identify random nodes, given certain
 * conditions. Usually these are used to identify origin and destination nodes
 * for possible trips.
 *
 */
public class NodesLookup {

	/**
	 * Given a graph, it returns a random node within it.
	 *
	 * @param network a graph;
	 */
	public static NodeGraph randomNode(Graph network) {
		final Random random = new Random();
		final Integer c = random.nextInt(network.nodesMap.values().size());
		final ArrayList<NodeGraph> nodes = new ArrayList<>(network.nodesMap.values());
		return nodes.get(c);
	}

	/**
	 * Given a graph and a list of nodes' geometries, it returns a random node of
	 * the graph, that is also contained in the list.
	 *
	 * @param network         a graph;
	 * @param nodesGeometries the list of nodes' geometries;
	 */
	public static NodeGraph randomNode(Graph network, ArrayList<MasonGeometry> nodesGeometries) {
		final Random random = new Random();
		final Integer c = random.nextInt(nodesGeometries.size());
		final MasonGeometry geoNode = nodesGeometries.get(c);
		return network.findNode(geoNode.geometry.getCoordinate());
	}

	/**
	 * Given a graph, the function returns a random node, within a certain radius
	 * from a given origin node and outside the given node's region.
	 *
	 * @param network    a graph;
	 * @param originNode a node;
	 * @param radius     the distance from the originNode, within which the random
	 *                   node should be found;
	 */
	public static NodeGraph randomNodeOtherRegion(Graph network, NodeGraph originNode, double radius) {

		final VectorLayer junctionsWithin = new VectorLayer();
		final MasonGeometry nodeGeometry = originNode.masonGeometry;
		final Random random = new Random();

		Bag filterSpatial = null;
		Bag filterByDistrict = null;
		NodeGraph n = null;
		double expanding_radius = radius;
		while (n == null) {
			if (expanding_radius >= radius * 2.00)
				return null;
			filterSpatial = network.junctions.featuresWithinDistance(nodeGeometry.geometry, expanding_radius);

			if (filterSpatial.size() < 1) {
				expanding_radius = expanding_radius * 1.10;
				continue;
			}
			for (final Object o : filterSpatial)
				junctionsWithin.addGeometry((MasonGeometry) o);

			if (junctionsWithin.getGeometries().size() == 0) {
				expanding_radius = expanding_radius * 1.10;
				continue;
			}

			filterByDistrict = junctionsWithin.filterFeatures("district", originNode.region, false);
			if (filterByDistrict.size() == 0) {
				expanding_radius = expanding_radius * 1.10;
				continue;
			}

			final Integer c = random.nextInt(filterByDistrict.size());
			final MasonGeometry geoNode = (MasonGeometry) filterByDistrict.objs[c];
			n = network.findNode(geoNode.geometry.getCoordinate());
			expanding_radius = expanding_radius * 1.10;
		}
		return n;
	}

	/**
	 * Given a graph, the function returns a random node that is approximately as
	 * far away from a given origin node, as a distance extracted from a list of
	 * distances.
	 *
	 * @param network    a graph;
	 * @param originNode a node;
	 * @param distances  the list of possible distances used to identify the node;
	 */
	public static NodeGraph randomNodeFromDistancesSet(Graph network, NodeGraph originNode, List<Float> distances) {

		final Random random = new Random();
		final int pD = random.nextInt(distances.size());
		double distance = distances.get(pD);
		if (distance < 100)
			distance = 100;
		NodeGraph node = null;
		ArrayList<NodeGraph> candidates = new ArrayList<>();
		double tolerance = 50;

		while (true) {
			final double lowerLimit = distance - tolerance;
			final double upperLimit = distance + tolerance;
			candidates = network.getNodesBetweenLimits(originNode, lowerLimit, upperLimit);
			if (candidates.size() > 1)
				break;
			else
				tolerance += 50;
		}
		while (node == null || node.getID() == originNode.getID()) {
			final Integer c = random.nextInt(candidates.size());
			node = candidates.get(c);
			if (originNode.getEdgeWith(node) != null)
				node = null;
		}
		return node;
	}

	/**
	 * Given a graph, the function returns a random node whose distance from a
	 * passed origin node is within certain limits.
	 *
	 * @param network    a graph;
	 * @param originNode a node;
	 * @param lowerLimit the minimum distance from the origin node;
	 * @param upperLimit the maximum distance from the origin node;
	 */
	public static NodeGraph randomNodeBetweenLimits(Graph network, NodeGraph originNode, double lowerLimit,
			double upperLimit) {
		final Random random = new Random();
		final ArrayList<NodeGraph> candidates = network.getNodesBetweenLimits(originNode, lowerLimit, upperLimit);
		final int c = random.nextInt(candidates.size());
		final NodeGraph node = candidates.get(c);
		return node;
	}

	/**
	 * Given a graph, the function returns a random node whose distance from a
	 * passed origin node is within certain limits. The returned node belongs to a
	 * region different from the origin node's region.
	 *
	 * @param network    a graph;
	 * @param originNode a node;
	 * @param lowerLimit the minimum distance from the origin node;
	 * @param upperLimit the maximum distance from the origin node;
	 */
	public static NodeGraph randomNodeBetweenLimitsOtherRegion(Graph network, NodeGraph originNode, double lowerLimit,
			double upperLimit) {
		final Random random = new Random();
		final ArrayList<NodeGraph> candidates = network.getNodesBetweenLimitsOtherRegion(originNode, lowerLimit,
				upperLimit);
		final int c = random.nextInt(candidates.size());
		final NodeGraph node = candidates.get(c);
		return node;
	}

	/**
	 * Given a graph, the function returns a random node whose distance from a
	 * passed origin node is within certain limits. The returned node belongs to a
	 * region different from the origin node's region.
	 *
	 * @param network    a graph;
	 * @param originNode a node;
	 * @param lowerLimit the minimum distance from the origin node;
	 * @param upperLimit the maximum distance from the origin node;
	 */
	public static NodeGraph randomSalientNodeBetweenLimits(Graph network, NodeGraph originNode, double lowerLimit,
			double upperLimit, double percentile) {

		final Random random = new Random();
		NodeGraph node = null;
		while (node == null) {
			final ArrayList<NodeGraph> candidates = network.getSalientNodesBetweenLimits(originNode, lowerLimit,
					upperLimit, percentile);
			final int c = random.nextInt(candidates.size());
			node = candidates.get(c);
			percentile -= 0.05;
			if (percentile == 0.0)
				return null;
		}
		return node;
	}

	/**
	 * Given a graph, the function returns a random node whose distance from a
	 * passed origin node is within certain limits. The returned node belongs to a
	 * region different from the origin node's region.
	 *
	 * @param network    a graph;
	 * @param originNode a node;
	 * @param lowerLimit the minimum distance from the origin node;
	 * @param upperLimit the maximum distance from the origin node;
	 */
	public static NodeGraph randomNodeDMA(Graph network, NodeGraph originNode, double lowerLimit, double upperLimit,
			String DMA) {

		final Random random = new Random();
		NodeGraph node = null;
		final double tolerance = 50.0;

		while (node == null) {
			ArrayList<NodeGraph> candidates = new ArrayList<>();
			if (originNode != null)
				candidates = network.getNodesBetweenLimits(originNode, lowerLimit, upperLimit);
			else
				candidates = network.getNodes();

			ArrayList<NodeGraph> candidatesDMA = new ArrayList<>();
			if (DMA.equals("random"))
				candidatesDMA = new ArrayList<>(candidates);
			else
				for (final NodeGraph n : candidates)
					if (n.DMA.equals(DMA))
						candidatesDMA.add(n);

			if (candidatesDMA.size() == 0) {
				if (upperLimit > UserParameters.maxDistance * 1.50) {
					final int c = random.nextInt(candidates.size());
					node = candidates.get(c);
					break;
				}
				upperLimit += tolerance;
				continue;
			}

			final int c = random.nextInt(candidatesDMA.size());
			node = candidatesDMA.get(c);
		}
		return node;
	}
}
