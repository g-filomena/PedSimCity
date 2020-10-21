package sim.app.geo.pedSimCity;

public class ResearchParameters {

	// Some researcher-defined parameter
	static double regionBasedNavigationThreshold = 600; //Region-based navigation Threshold - meters
	static double visibilityThreshold = 300; //2d Advance Visibility threshold; distanLandmarks usage threshold
	static double wayfindingEasinessThreshold = 0.95; //2d Visibility threshold; distanLandmarks usage threshold
	static double globalLandmarknessWeight = 0.80; //weight Global Landmarkness in combination with edge cost
	static double globalLandmarkThreshold = 0.20; //
	static double localLandmarkThreshold = 0.30; //
	static double salientNodesPercentile = 0.75; // Threshold Percentile to identify salient nodes


	static double noobAgentThreshold = 0.25;
	static double expertAgentThreshold = 0.75;
}
