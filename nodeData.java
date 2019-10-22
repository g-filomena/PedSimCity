package sim.app.geo.pedestrianSimulation;

import java.util.List;

import com.vividsolutions.jts.planargraph.Node;

public class nodeData {
	
	Node node;
	int district;
	double bC;
	boolean gateway;
	
	List<Integer> visible2d;
	
	List<Integer> localLandmarks;
	List<Double> localScores;
	
	List<Integer> distantLandmarks;
	List<Double> distantScores;
	
	List<Integer> anchors;
	List<Double> distances;
}
