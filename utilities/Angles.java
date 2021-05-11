package pedsimcity.utilities;

import com.vividsolutions.jts.geom.Coordinate;
import com.vividsolutions.jts.geom.Geometry;
import com.vividsolutions.jts.geom.GeometryFactory;
import com.vividsolutions.jts.geom.MultiPoint;

import pedsimcity.graph.NodeGraph;

/**
 * Set of functions for computing angles' and related-measures.
 *
 */

public class Angles {

	private static double dot(double[] vectorA, double[] vectorB) {
		return vectorA[0] * vectorB[0] + vectorA[1] * vectorB[1];
	}

	/**
	 * It computes the angle formed by two nodes in a network, with the y-axis It
	 * returns a value in degrees.
	 *
	 * @param originNode      the first node;
	 * @param destinationNode the second node;
	 */
	public static double angle(NodeGraph originNode, NodeGraph destinationNode) {
		final Coordinate origin = originNode.getCoordinate();
		final Coordinate destination = destinationNode.getCoordinate();
		final double[] vectorA = { origin.x - origin.x, origin.y - (origin.y + 2000) };
		final double[] vectorB = { origin.x - destination.x, origin.y - destination.y };
		final double dot_prod = dot(vectorA, vectorB);
		final double magA = Math.pow(dot(vectorA, vectorA), 0.5);
		final double magB = Math.pow(dot(vectorB, vectorB), 0.5);

		final double anglRad = Math.acos(dot_prod / magB / magA);
		double angleDeg = Math.toDegrees(anglRad) % 360;
		if (destination.x < origin.x)
			angleDeg = 180 + (180 - angleDeg);
		return angleDeg;
	}

	/**
	 * It computes the angle formed by two locations, expressed in coordinates
	 * pairs, with the y-axis. It returns a value in degrees.
	 *
	 * @param origin      the first coordinates pair;
	 * @param destination the second coordinates pair;
	 */
	public static double angle(Coordinate origin, Coordinate destination) {
		final double[] vectorA = { origin.x - origin.x, origin.y - (origin.y + 2000) };
		final double[] vectorB = { origin.x - destination.x, origin.y - destination.y };
		final double dot_prod = dot(vectorA, vectorB);
		final double magA = Math.pow(dot(vectorA, vectorA), 0.5);
		final double magB = Math.pow(dot(vectorB, vectorB), 0.5);

		final double anglRad = Math.acos(dot_prod / magB / magA);
		double angleDeg = Math.toDegrees(anglRad) % 360;
		if (destination.x < origin.x)
			angleDeg = 180 + (180 - angleDeg);
		return angleDeg;

	}

	/**
	 * It computes the difference between two angles. It returns a value in degrees.
	 *
	 * @param angleA the first angle (degrees);
	 * @param angleB the second angle (degrees);
	 */
	public static double differenceAngles(Double angleA, Double angleB) {
		double difference = 0;
		// check if same quadrant
		if (angleA <= 180 & angleB <= 180 || angleA > 180 & angleB > 180)
			difference = Math.abs(angleA - angleB);

		else if (angleA > 180 & angleB <= 180) {
			final double tmpA = Math.abs(angleB - angleA);
			final double tmpB = Math.abs(angleA - (angleB + 360));
			difference = Math.min(tmpA, tmpB);
		}
		// (angleB > 180 & angleA <= 180)
		else {
			final double tmpA = Math.abs(angleB - angleA);
			final double tmpB = Math.abs(angleB - (angleA + 360));
			difference = Math.min(tmpA, tmpB);
		}
		return difference;
	}

	/**
	 * It verifies whether an angle between an origin and a second node is towards
	 * certain destination, on the basis of a cone of x degrees.
	 *
	 * @param angleOD the angle between the origin and the destination (degrees);
	 * @param angleON the second angle, between the origin and a possible
	 *                intermediate node (degrees);
	 * @param cone    the amplitude of the cone (degrees);
	 */
	public static boolean isInDirection(double angleOD, double angleON, double cone) {
		double limitLeft = angleOD - (cone / 2.0 + 1.0);
		double limitRight = angleOD + (cone / 2.0 + 1.0);

		if (limitLeft < 0.0)
			limitLeft = 360.0 + limitLeft;
		if (limitRight > 360.0)
			limitRight = limitRight - 360.0;

		// over the 0
		if (limitLeft > limitRight) {
			if (angleON >= limitLeft)
				return true;
		} else if (limitLeft < limitRight)
			if (angleON >= limitLeft && angleON <= limitRight)
				return true;
		return false;
	}

	/**
	 * Given a NodeGraph, a distance from it, and a desired angle formed with the y
	 * axis, it returns the coordinates of the resulting location.
	 *
	 * @param originNode the input node;
	 * @param distance   the distance from the input node;
	 * @param angle      the angle formed by the angle with the y axis;
	 */
	public static Coordinate getCoordAngle(NodeGraph originNode, double distance, double angle) {
		final double x = distance * Math.sin(Math.toRadians(angle)) + originNode.getCoordinate().x;
		final double y = distance * Math.cos(Math.toRadians(angle)) + originNode.getCoordinate().y;
		final Coordinate coord = new Coordinate(x, y);
		return coord;
	}

	/**
	 * Given two NodeGraph, and half of the amplitude of a desired angle, either
	 * positive or negative (+ 35.0° or - 35.0° --> 70°) from the first node towards
	 * the other one, it returns the coordinate of the point that would form with
	 * the first node one of the lines of the view cone.
	 *
	 * @param node         a node;
	 * @param otherNode    another node;
	 * @param desiredAngle half of the field of view's amplitude;
	 */
	private static Coordinate angleViewField(NodeGraph node, NodeGraph otherNode, double desiredAngle) {
		double angle = angle(node, otherNode);

		if (angle > 360)
			angle = angle - 360.0;
		double resultingAngle = angle + desiredAngle;
		if (resultingAngle > 360.0)
			resultingAngle = resultingAngle - 360.0;
		final Coordinate coord = getCoordAngle(node, NodeGraph.nodesDistance(node, otherNode), resultingAngle);
		return coord;
	}

	/**
	 * Given two NodeGraph, and the amplitude of a desired field (or cone) of view
	 * from the first node towards the second node, it returns the corresponding
	 * geometry.
	 *
	 * @param node        a node;
	 * @param otherNode   another node;
	 * @param fieldOfView the desired field of view's amplitude;
	 */
	public static Geometry viewField(NodeGraph node, NodeGraph otherNode, double fieldOfView) {

		if (fieldOfView >= 180.0)
			fieldOfView = 140.0;

		final Coordinate coordA = angleViewField(node, otherNode, fieldOfView / 2.0);
		final Coordinate coordB = angleViewField(node, otherNode, -fieldOfView / 2.0);
		final GeometryFactory geometryFactory = new GeometryFactory();
		final MultiPoint points = geometryFactory
				.createMultiPoint(new Coordinate[] { node.getCoordinate(), coordA, coordB, otherNode.getCoordinate() });

		final Geometry viewField = points.convexHull();
		return viewField;
	}

}
