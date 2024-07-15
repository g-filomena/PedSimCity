package pedSim.engine;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Paths;
import java.time.LocalDate;
import java.time.format.DateTimeFormatter;
import java.util.ArrayList;
import java.util.List;
import java.util.Map;

import org.apache.commons.lang3.ArrayUtils;

import pedSim.agents.EmpiricalAgentsGroup;
import pedSim.utilities.RouteData;
import pedSim.utilities.StringEnum;
import pedSim.utilities.StringEnum.RouteChoice;
import sim.field.geo.VectorLayer;
import sim.io.geo.ShapeFileExporter;
import sim.util.geo.CSVUtils;
import sim.util.geo.MasonGeometry;

/**
 * The Export class is responsible for saving the simulation results to
 * specified output directories.
 */
public class Exporter {

	private String userName = System.getProperty("user.name");
	// Constants for file paths and directories
	public String outputDirectory;
	public String outputRoutesDirectory;
	public String outputVolumesDirectory;

	private static int nrColumns;
	private static final int FIELD_LIMIT = 254;
	int job;
	FlowHandler flowHandler;
	private String currentDate;

	public Exporter(FlowHandler flowHandler) {
		this.flowHandler = flowHandler;
		this.job = flowHandler.job;
		outputDirectory = "C:" + File.separator + "Users" + File.separator + userName + File.separator
				+ "PedSimCitySocial" + File.separator + "Output";
		DateTimeFormatter formatter = DateTimeFormatter.ofPattern("yyyyMMdd");
		currentDate = LocalDate.now().format(formatter);
	}

	/**
	 * Saves pedestrian volumes data to a CSV file.
	 *
	 * @param job The job identifier.
	 * @throws Exception If there is an error while saving the data.
	 */
	public void saveVolumes() throws Exception {

		String specifier = findSpecifier() + "_streetVolumes";
		outputVolumesDirectory = verifyOutputPath(outputVolumesDirectory, specifier);
		outputVolumesDirectory += File.separator + currentDate + ".csv";
		final FileWriter writerVolumesData = new FileWriter(outputVolumesDirectory);
		Map<Integer, Map<String, Integer>> volumesMap = flowHandler.volumesMap;
		List<String> headers = new ArrayList<>();
		headers.add("edgeID"); // Directly adding "edgeID" without specifying index

		if (Parameters.empirical)
			for (final EmpiricalAgentsGroup group : PedSimCity.empiricalGroups)
				headers.add(group.groupName.toString());
		else {
			List<String> abbreviations = new ArrayList<>();
			for (RouteChoice routeChoice : Parameters.routeChoiceModels)
				abbreviations.add(StringEnum.getAbbreviation(routeChoice));

			headers = new ArrayList<>(abbreviations);
		}
		CSVUtils.writeLine(writerVolumesData, headers);

		for (int edgeID : volumesMap.keySet()) {
			Map<String, Integer> edgeVolumes = volumesMap.get(edgeID);
			List<String> row = new ArrayList<>();
			// Adding edgeID as the first column value
			row.add(Integer.toString(edgeID));

			for (String columnHeader : headers.subList(1, headers.size())) { // Skip "edgeID" header
				Integer volume = edgeVolumes.get(columnHeader);
				row.add(volume != null ? Integer.toString(volume) : "0"); // Handle potential null values ;
			}
			CSVUtils.writeLine(writerVolumesData, row);
		}
		writerVolumesData.flush();
		writerVolumesData.close();
	}

	/**
	 * Saves pedestrian volumes data to a CSV file.
	 *
	 * @param job The job identifier.
	 * @throws Exception If there is an error while saving the data.
	 */
	public void saveRoutes() throws Exception {

		String specifier = findSpecifier() + "_routes";
		outputRoutesDirectory = verifyOutputPath(outputRoutesDirectory, specifier);
		outputRoutesDirectory += File.separator + currentDate + "_" + job;
		VectorLayer routes = new VectorLayer();
		nrColumns = 0;

		for (RouteData routeData : flowHandler.routesData) {
			MasonGeometry masonGeometry = new MasonGeometry(routeData.lineGeometry);
			masonGeometry.addIntegerAttribute("O", routeData.origin);
			masonGeometry.addIntegerAttribute("D", routeData.destination);
			if (Parameters.empirical)
				updateEmpiricalRouteData(routeData, masonGeometry);
			else
				masonGeometry.addStringAttribute("attribute", routeData.scenario);

			formRouteAttributes(masonGeometry, routeData);
			routes.addGeometry(masonGeometry);
		}

		// This is to avoid having geometries without the needed columns' values filled
		// in.
		if (nrColumns > 0) {

			for (int counter = 1; counter <= nrColumns - 1; counter++) {
				List<MasonGeometry> routeGeometries = routes.getGeometries();
				for (MasonGeometry route : routeGeometries)
					if (route.hasAttribute("edgeIDs_" + counter))
						continue;
					else
						route.addAttribute("edgeIDs_" + counter, "None");
			}
		}
		ShapeFileExporter.write(outputRoutesDirectory, routes);
	}

	private String findSpecifier() {
		String specifier = null;
		if (Parameters.empirical)
			specifier = "empirical";
		else if (Parameters.testingSubdivisions)
			specifier = "subdivisions";
		else if (Parameters.testingLandmarks)
			specifier = "landmarkNavigation";
		else if (Parameters.testingModels)
			specifier = "testing";
		return specifier;
	}

	/**
	 * Forms route attributes and handles splitting long edgeIDs strings.
	 *
	 * @param masonGeometry The MasonGeometry object representing a route.
	 * @param routeData     The route data associated with the route.
	 */
	private static void formRouteAttributes(MasonGeometry masonGeometry, RouteData routeData) {
		String edgeIDs = ArrayUtils.toString(routeData.edgeIDsSequence);

		if (edgeIDs.length() <= FIELD_LIMIT)
			masonGeometry.addAttribute("edgeIDs_0", edgeIDs);
		else {
			String remainingEdges = edgeIDs;
			for (int counter = 0; remainingEdges.length() > 0; counter++) {
				if (counter >= nrColumns)
					nrColumns += 1;

				String currentPart;
				if (remainingEdges.length() > FIELD_LIMIT) {
					currentPart = remainingEdges.substring(0, FIELD_LIMIT);
					remainingEdges = remainingEdges.substring(FIELD_LIMIT);
				} else {
					currentPart = remainingEdges;
					masonGeometry.addAttribute("edgeIDs_" + counter, currentPart);
					break;
				}
				masonGeometry.addAttribute("edgeIDs_" + counter, currentPart);
			}
		}
	}

	/**
	 * Verifies and creates the specified output directory.
	 * 
	 * @param directory The directory path to be created.
	 * @return
	 */
	private String verifyOutputPath(String directory, String specifier) {

		directory = outputDirectory + File.separator + specifier;
		directory = String.format(directory, userName);

		File outputCheck = new File(directory);
		if (!outputCheck.exists()) {
			try {
				// Create the output path directory and its parent directories recursively
				Files.createDirectories(Paths.get(directory));
			} catch (IOException e) {
				e.printStackTrace();
			}
		}
		return directory;
	}

	/**
	 * Forms route attributes and handles splitting long edgeIDs strings.
	 *
	 * @param masonGeometry The MasonGeometry object representing a route.
	 * @param routeData     The route data associated with the route.
	 */
	private static void updateEmpiricalRouteData(RouteData routeData, MasonGeometry masonGeometry) {
		masonGeometry.addStringAttribute("group", routeData.group);
		masonGeometry.addStringAttribute("min", getMinimisation(routeData));
		masonGeometry.addStringAttribute("heur", getHeuristic(routeData));
		masonGeometry.addIntegerAttribute("regions", routeData.regionBased ? 1 : 0);
		masonGeometry.addIntegerAttribute("routeMark", routeData.onRouteMarks ? 1 : 0);
		masonGeometry.addIntegerAttribute("barrier", routeData.barrierSubGoals ? 1 : 0);
		masonGeometry.addIntegerAttribute("distant", routeData.distantLandmarks ? 1 : 0);
		masonGeometry.addDoubleAttribute("natural", routeData.naturalBarriers);
		masonGeometry.addDoubleAttribute("severing", routeData.severingBarriers);
	}

	/**
	 * Gets the minimisation type as a string.
	 *
	 * @param routeData The route data.
	 * @return The minimisation type as a string or null if not applicable.
	 */
	private static String getMinimisation(RouteData routeData) {
		if (routeData.minimisingDistance)
			return "distance";
		else if (routeData.minimisingAngular)
			return "angular";
		else
			return "none";
	}

	/**
	 * Transforms the local minimisation heuristic as a string.
	 *
	 * @param routeData The route data.
	 * @return The minimisation heuristic as a string or null if not applicable.
	 */
	private static String getHeuristic(RouteData routeData) {
		if (routeData.localHeuristicDistance)
			return "distance";
		else if (routeData.localHeuristicAngular)
			return "angular";
		else
			return "none";
	}
}
