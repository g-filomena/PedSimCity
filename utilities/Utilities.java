package pedsimcity.utilities;

import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;
import java.util.LinkedHashMap;
import java.util.Map;
import java.util.Map.Entry;
import java.util.Objects;
import java.util.Random;
import java.util.stream.Collectors;

import com.vividsolutions.jts.geom.Coordinate;

import sim.util.Bag;
import sim.util.geo.MasonGeometry;

/**
 * A class of various utilities.
 *
 */
public class Utilities {

	/** It sorts a Map on the basis of its values, and on the method provided (descending)
	 *
	 * @param map the map;
	 * @param descending if true, values are sorted in a descending order, otherwise ascending;
	 */
	public static <K, V extends Comparable<? super V>> Map<K, V> sortByValue(Map<K, V> map, boolean descending) {
		if (descending) return map.entrySet()
				.stream()
				.sorted(Map.Entry.comparingByValue(Collections.reverseOrder()))
				.collect(Collectors.toMap(
						Map.Entry::getKey,
						Map.Entry::getValue,
						(e1, e2) -> e1,
						LinkedHashMap::new
						));

		else return map.entrySet()
				.stream()
				.sorted(Map.Entry.comparingByValue())
				.collect(Collectors.toMap(
						Map.Entry::getKey,
						Map.Entry::getValue,
						(e1, e2) -> e1,
						LinkedHashMap::new
						));
	}

	/**
	 * It computes the Euclidean distance between two locations
	 *
	 * @param originCoord the origin location;
	 * @param destinationCoord the destination;
	 */
	public static double euclideanDistance(Coordinate originCoord, Coordinate destinationCoord) {
		return Math.sqrt(Math.pow(originCoord.x - destinationCoord.x, 2)
				+ Math.pow(originCoord.y - destinationCoord.y, 2));
	}

	/**
	 * It filters the content of the map on the basis of the key values. Only entries whose keys are contained in the given Bag are kept.
	 * A new map is returned.
	 *
	 * @param map, the input map;
	 * @param filter, a Bag containing the desired keys;
	 */
	public static HashMap<MasonGeometry, Double> filterMap(HashMap<MasonGeometry, Double> map, Bag filter) {
		HashMap<MasonGeometry, Double> mapFiltered = new HashMap<MasonGeometry, Double> (map);
		ArrayList<MasonGeometry> result = new ArrayList<MasonGeometry>();
		for(MasonGeometry key : mapFiltered.keySet()) {if(filter.contains(key)) result.add(key);}
		mapFiltered.keySet().retainAll(result);
		return mapFiltered;
	}

	/**
	 * Given a certain value, it returns the key of the first entry whose values is equal to the input.
	 *
	 * @param map, the input map;
	 * @param value, the input value whose key is of interest;
	 */
	public static <K, V> K getKeyFromValue(Map<K, V> map, V value) {
		for (Entry<K, V> entry : map.entrySet()) {
			if (Objects.equals(value, entry.getValue())) return entry.getKey();
		}
		return null;
	}

	/**
	 * It returns a random value from a distribution with a given mean and a standard deviation.
	 * If a direction is provided, only values higher or lower than the mean are returned.
	 *
	 * @param mean the distribution's mean;
	 * @param sd the distribution's standard deviation;
	 * @param direction either "left", only values lower than the mean, or "right", higher than the mean, otherwise pass null;
	 */
	public static double fromDistribution(double mean, double sd, String direction) 	{
		Random random = new Random();
		double result = random.nextGaussian()*sd+mean;
		if (direction != null) {
			if ((direction.equals("left")) && (result > mean)) result = mean;
			if ((direction.equals("right")) && (result < mean)) result = mean;
		}
		if (result <= 0.00) result = mean;
		return result;
	}

}




