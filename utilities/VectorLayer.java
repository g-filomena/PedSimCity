package pedsimcity.utilities;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import com.vividsolutions.jts.algorithm.ConvexHull;
import com.vividsolutions.jts.geom.Coordinate;
import com.vividsolutions.jts.geom.Envelope;
import com.vividsolutions.jts.geom.Geometry;
import com.vividsolutions.jts.geom.GeometryFactory;
import com.vividsolutions.jts.index.quadtree.Quadtree;

import sim.field.geo.GeomVectorField;
import sim.util.Bag;
import sim.util.geo.MasonGeometry;

/**
 * This class extends GeomVectorField and provides further geometric, selection
 * and filter functions.
 *
 */

public class VectorLayer extends GeomVectorField {

	private static final long serialVersionUID = 1L;
	public ArrayList<MasonGeometry> geometriesList = new ArrayList<>();
	private Quadtree layerSpatialIndex = new Quadtree();
	private final GeometryFactory layerGeomFactory = new GeometryFactory();

	public VectorLayer() {
		super();

	}

	/**
	 * It creates a VectorLayer given a Bag containing MasonGeometry objects.
	 *
	 * @param geometries a Bag containing MasonGeometry objects;
	 */
	public VectorLayer(Bag geometries) {
		super();

		for (final Object o : geometries) {
			final MasonGeometry mg = (MasonGeometry) o;
			super.addGeometry(mg);
			final Envelope e = mg.getGeometry().getEnvelopeInternal();
			this.layerSpatialIndex.insert(e, mg);
		}
		this.generateGeometriesList();
	}

	@Override
	public void addGeometry(MasonGeometry mg) {
		super.addGeometry(mg);
		final Envelope e = mg.getGeometry().getEnvelopeInternal();
		this.layerSpatialIndex.insert(e, mg);
	}

	/**
	 * It returns all the geometries in the VectorLayer that intersect a given
	 * Geometry.
	 *
	 * @param inputGeometry the geometry on which the intersection should be based;
	 */
	public final Bag intersectingFeatures(Geometry inputGeometry) {

		final Bag intersectingObjects = new Bag();
		final Envelope e = inputGeometry.getEnvelopeInternal();
		e.expandBy(java.lang.Math.max(e.getHeight(), e.getWidth()) * 0.01);
		final List<?> gList = this.layerSpatialIndex.query(e);

		for (final Object o : gList) {
			final MasonGeometry mg = (MasonGeometry) o;
			if (inputGeometry.intersects(mg.geometry))
				intersectingObjects.add(mg);
		}
		return intersectingObjects;
	}

	/**
	 * It returns all the geometries in the VectorLayer that are contained by a
	 * given Geometry.
	 *
	 * @param inputGeometry the geometry on which the containment relationship
	 *                      should be based;
	 */
	public final ArrayList<MasonGeometry> containedFeatures(Geometry inputGeometry) {
		final ArrayList<MasonGeometry> containedObjects = new ArrayList<>();
		final Envelope e = inputGeometry.getEnvelopeInternal();
		e.expandBy(java.lang.Math.max(e.getHeight(), e.getWidth()) * 0.01);
		final List<?> gList = this.layerSpatialIndex.query(e);

		for (final Object o : gList) {
			final MasonGeometry mg = (MasonGeometry) o;
			if (inputGeometry.contains(mg.geometry))
				containedObjects.add(mg);
		}
		return containedObjects;
	}

	/**
	 * It returns all the geometries in the VectorLayer that are contained within a
	 * certain space from a given Geometry.
	 *
	 * @param inputGeometry the geometry on which the containment relationship
	 *                      should be based;
	 * @param lowerLimit    the minimum distance from the input geometry;
	 * @param upperLimit    the maximum distance from the input geometry;
	 */
	public Bag featuresBetweenLimits(Geometry inputGeometry, double lowerLimit, double upperLimit) {
		final Bag objects = new Bag();
		final Envelope e = inputGeometry.getEnvelopeInternal();
		e.expandBy(upperLimit);
		final List<?> gList = this.layerSpatialIndex.query(e);

		for (final Object o : gList) {
			final MasonGeometry mg = (MasonGeometry) o;
			if (inputGeometry.distance(mg.geometry) >= lowerLimit
					& inputGeometry.distance(mg.getGeometry()) <= upperLimit)
				objects.add(mg);
			else
				continue;
		}
		return objects;
	}

	/**
	 * It returns all the geometries in the VectorLayer that are contained within a
	 * certain radius from a given Geometry.
	 *
	 * @param inputGeometry the geometry on which the containment relationship
	 *                      should be based;
	 * @param radius        the distance from the input geometry;
	 */
	public Bag featuresWithinDistance(Geometry inputGeometry, double radius) {
		final Bag nearbyObjects = new Bag();
		final Envelope e = inputGeometry.getEnvelopeInternal();
		e.expandBy(radius);

		final List<?> gList = this.layerSpatialIndex.query(e);

		for (final Object o : gList) {
			final MasonGeometry mg = (MasonGeometry) o;
			if (inputGeometry.isWithinDistance(mg.geometry, radius))
				nearbyObjects.add(mg);
		}

		return nearbyObjects;
	}

	/**
	 * It returns all the geometries in the VectorLayer whose (Integer) attribute's
	 * value is equal or different to the provided.
	 *
	 * @param attributeName  the field's name on which the equality is verified;
	 * @param attributeValue the desired (or not) value;
	 * @param equal          if true, features with values equal to the input
	 *                       attributeValue are kept; if false, all the other ones.
	 */
	public Bag filterFeatures(String attributeName, int attributeValue, boolean equal) {

		final Bag objects = new Bag();

		for (final Object o : this.getGeometries()) {
			final MasonGeometry mg = (MasonGeometry) o;
			final Integer attribute = mg.getIntegerAttribute(attributeName);
			if (!equal && !attribute.equals(attributeValue))
				objects.add(mg);
			else if (attribute.equals(attributeValue))
				objects.add(mg);
		}
		return objects;
	}

	/**
	 * It returns all the geometries in the VectorLayer whose (String) attribute's
	 * value is equal or different to the provided.
	 *
	 * @param attributeName  the field's name on which the equality is verified;
	 * @param attributeValue the desired (or not) value;
	 * @param equal          if true, features with values equal to the input
	 *                       attributeValue are returned; if false, all the other
	 *                       ones.
	 */
	public Bag filterFeatures(String attributeName, String attributeValue, boolean equal) {
		final Bag objects = new Bag();

		for (final Object o : this.getGeometries()) {
			final MasonGeometry mg = (MasonGeometry) o;
			final String attribute = mg.getStringAttribute(attributeName);
			if (!equal && !attribute.equals(attributeValue))
				objects.add(mg);
			else if (attribute.equals(attributeValue))
				objects.add(mg);
		}
		return objects;
	}

	/**
	 * It returns all the geometries in the VectorLayer whose (String) attribute's
	 * value is contained in a provided list of values.
	 *
	 * @param attributeName the field's name on which the equality is verified;
	 * @param listValues    the list of desired (or not) values;
	 * @param equal         if true, features with values contained in the input
	 *                      listValues are kept; if false, all the other ones.
	 */
	public Bag filterFeatures(String attributeName, List<String> listValues, boolean equal) {
		final Bag objects = new Bag();

		for (final Object o : this.getGeometries()) {
			final MasonGeometry mg = (MasonGeometry) o;
			final String attribute = mg.getStringAttribute(attributeName);
			if (!equal && !listValues.contains(attribute))
				objects.add(mg);
			else if (listValues.contains(attribute))
				objects.add(mg);
		}
		return null;
	}

	/**
	 * Given a field name in the VectorLayer, it returns the list of values.
	 *
	 * @param attributeName the field's name;
	 */
	public List<Integer> getIntColumn(String attributeName) {
		final List<Integer> values = new ArrayList<>();

		for (final Object o : this.getGeometries()) {
			final MasonGeometry mg = (MasonGeometry) o;
			final Integer attribute = mg.getIntegerAttribute(attributeName);
			values.add(attribute);
		}
		return values;
	}

	/**
	 * Given a field name in the VectorLayer, and a list of values, it return a new
	 * VectorLayer only containing features with values contained in the list (when
	 * equal == True), or the complementary ones (equal == false). it returns the
	 * list of values.
	 *
	 * @param attributeName the field's name;
	 * @param listValues    the list of desired (or not) values;
	 * @param equal         if true, features with values contained in the input
	 *                      listValues are kept; if false, all the other ones.
	 */
	public VectorLayer selectFeatures(String attributeName, List<Integer> listValues, boolean equal) {
		final Bag objects = new Bag();

		for (final Object o : this.getGeometries()) {
			final MasonGeometry mg = (MasonGeometry) o;
			final Integer attribute = mg.getIntegerAttribute(attributeName);
			if (!equal && !listValues.contains(attribute))
				objects.add(mg);
			else if (equal && listValues.contains(attribute))
				objects.add(mg);
		}
		final VectorLayer newLayer = new VectorLayer(objects);
		return newLayer;
	}

	/**
	 * It computes the VectorLayer's convexHull
	 *
	 */
	public Geometry layerConvexHull() {
		final ArrayList<Coordinate> pts = new ArrayList<>();

		for (final Object o : this.getGeometries()) {
			final Geometry g = ((MasonGeometry) o).geometry;
			final Coordinate c[] = g.getCoordinates();
			pts.addAll(Arrays.asList(c));
		}

		final Coordinate[] coords = pts.toArray(new Coordinate[pts.size()]);
		final ConvexHull convexHull = new ConvexHull(coords, this.layerGeomFactory);
		return convexHull.getConvexHull();
	}

	/**
	 * It sets the ID of this layer, for retrieval in other functions or during the
	 * simulation, given a field name. The ID is passed to the geometries as
	 * "userData".
	 *
	 * @param attributeName the field's name;
	 */
	public void setID(String attributeName) {

		for (final Object o : this.getGeometries()) {
			final MasonGeometry mg = (MasonGeometry) o;
			mg.setUserData(mg.getIntegerAttribute(attributeName));
		}
	}

	/**
	 * It returns a Bag of geometries containing all the elements of this
	 * VectorLayer that intersect another provided layer. When inclusive == false,
	 * the bag contains not intersecting geometries.
	 *
	 * @param otherLayer the other layer used to detect possible intersecting
	 *                   features;
	 * @param inclusive  if the intersecting features should be kept, or all the
	 *                   other ones;
	 */
	public Bag intersection(VectorLayer otherLayer, boolean inclusive) {

		final Bag intersecting = new Bag();
		for (final Object o : otherLayer.getGeometries()) {
			final Bag tmp = this.intersectingFeatures((Geometry) o);
			intersecting.addAll(tmp);
		}
		if (inclusive)
			return intersecting;
		else {
			final Bag notIntersecting = otherLayer.getGeometries();
			notIntersecting.removeAll(intersecting);
			return notIntersecting;
		}
	}

	/**
	 * It verifies if any of the features of this VectorLayer intersect a given
	 * geometry.
	 *
	 * @param inputGeometry the input geometry on which the intersection is
	 *                      verified;
	 */
	public boolean isIntersected(Geometry inputGeometry) {

		final Envelope e = inputGeometry.getEnvelopeInternal();
		e.expandBy(java.lang.Math.max(e.getHeight(), e.getWidth()) * 0.01);
		final List<?> gList = this.layerSpatialIndex.query(e);

		for (final Object o : gList) {
			final MasonGeometry mg = (MasonGeometry) o;
			if (inputGeometry.intersects(mg.geometry))
				return true;
		}
		return false;
	}

	/**
	 * It generates an ArrayList of MasonGeometry for this VectorLayer, for more
	 * straightforward iterations.
	 *
	 */
	public void generateGeometriesList() {

		this.geometriesList.clear();
		this.layerSpatialIndex = new Quadtree();
		for (final Object o : this.getGeometries()) {
			final MasonGeometry mg = (MasonGeometry) o;
			this.geometriesList.add(mg);
			final Envelope e = mg.getGeometry().getEnvelopeInternal();
			this.layerSpatialIndex.insert(e, mg);
		}
	}
}
