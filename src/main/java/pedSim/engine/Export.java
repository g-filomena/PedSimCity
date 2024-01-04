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
import java.util.logging.Logger;

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
public class Export {

	private String userName = System.getProperty("user.name");
	// Constants for file paths and directories
	public String outputDirectory;
	public String outputRoutesDirectory;
	private static final Logger LOGGER = Logger.getLogger(Export.class.getName());
	private static int nrColumns;
	private static final int FIELD_LIMIT = 254;
	int job;
	FlowHandler flowHandler;

	/**
	 * Saves simulation results to specified output directories.
	 *
	 * @throws Exception If there is an error while saving the results.
	 */
	public void saveResults(FlowHandler flowHandler) throws Exception {
		outputDirectory = "C:" + File.separator + "Users" + File.separator + userName + File.separator + "PedSimCity"
				+ File.separator + "Output";
		outputRoutesDirectory = outputDirectory;
		this.flowHandler = flowHandler;
		this.job = flowHandler.job;
		setOutputPath();
		savePedestrianVolumes();
		saveRoutes();
		LOGGER.info("Job nr " + job + ": Files successfully exported.");
	}

	/**
	 * Sets the output path based on the program's configuration.
	 */
	private void setOutputPath() {
		if (Parameters.empirical)
			outputDirectory += File.separator + "empirical";
		else if (Parameters.testingSubdivisions)
			outputDirectory += File.separator + "subdivisions";
		else if (Parameters.testingLandmarks)
			outputDirectory += File.separator + "landmarkNavigation";
		else if (Parameters.testingModels)
			outputDirectory += File.separator + "testing";

		outputRoutesDirectory = outputDirectory + File.separator + "routes";
		outputDirectory += File.separator + "streetVolumes";
		verifyOutputPath();
	}

	/**
	 * Verifies and creates the specified output directory.
	 */
	private void verifyOutputPath() {
		String username = System.getProperty("user.name");
		outputDirectory = String.format(outputDirectory, username);
		outputRoutesDirectory = String.format(outputRoutesDirectory, username);
		createDirectory(outputDirectory);
		createDirectory(outputRoutesDirectory);
	}

	/**
	 * Creates a directory if it does not exist.
	 *
	 * @param directory The directory path to be created.
	 */
	private void createDirectory(String directory) {

		File outputCheck = new File(directory);
		if (!outputCheck.exists()) {
			try {
				// Create the output path directory and its parent directories recursively
				Files.createDirectories(Paths.get(directory));
			} catch (IOException e) {
				e.printStackTrace();
			}
		}
	}

	/**
	 * Saves pedestrian volumes data to a CSV file.
	 *
	 * @param job The job identifier.
	 * @throws Exception If there is an error while saving the data.
	 */
	private void savePedestrianVolumes() throws Exception {

		DateTimeFormatter formatter = DateTimeFormatter.ofPattern("yyyyMMdd");
		String currentDate = LocalDate.now().format(formatter);
		final String csvSegments = outputDirectory + File.separator + currentDate + "_" + job + ".csv";
		final FileWriter writerVolumesData = new FileWriter(csvSegments);

		Map<Integer, Map<String, Integer>> volumesMap = flowHandler.volumesMap;

		ArrayList<String> headers;
		if (Parameters.empirical) {
			headers = new ArrayList<>();
			for (final EmpiricalAgentsGroup group : PedSimCity.empiricalGroups)
				headers.add(group.groupName.toString());
		} else {
			List<String> abbreviations = new ArrayList<>();
			for (RouteChoice routeChoice : Parameters.routeChoiceModels) {
				abbreviations.add(StringEnum.getAbbreviation(routeChoice));
			}
			headers = new ArrayList<>(abbreviations);
		}
		headers.add(0, "edgeID");
		CSVUtils.writeLine(writerVolumesData, headers);

		for (int edgeID : volumesMap.keySet()) {
			Map<String, Integer> edgeVolumes = volumesMap.get(edgeID);
			final List<String> row = new ArrayList<>();

			for (final String columnHeader : headers) {
				if (columnHeader.equals("edgeID")) {
					row.add(0, Integer.toString(edgeID));
					continue;
				}
				int index = headers.indexOf(columnHeader) - 1;
				String value = "";
				if (Parameters.empirical)
					value = headers.get(index + 1);
				else
					value = Parameters.routeChoiceModels[index].toString();
				row.add(Integer.toString(edgeVolumes.get(value)));
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
	private void saveRoutes() throws Exception {

		DateTimeFormatter formatter = DateTimeFormatter.ofPattern("yyyyMMdd");
		String currentDate = LocalDate.now().format(formatter);
		final VectorLayer routes = new VectorLayer();
		final String directory = outputRoutesDirectory + File.separator + currentDate + "_" + flowHandler.job;
		nrColumns = 0;

		for (final RouteData routeData : flowHandler.routesData) {
			MasonGeometry masonGeometry = new MasonGeometry(routeData.lineGeometry);
			masonGeometry.addIntegerAttribute("O", routeData.origin);
			masonGeometry.addIntegerAttribute("D", routeData.destination);
			if (Parameters.empirical)
				updateEmpiricalRouteData(routeData, masonGeometry);
			else
				masonGeometry.addStringAttribute("routeChoice", routeData.routeChoice);

			formRouteAttributes(masonGeometry, routeData);
			routes.addGeometry(masonGeometry);
		}

		// This is to avoid having geometries without the needed columns' values filled
		// in.
		if (nrColumns > 0) {

			for (int counter = 1; counter <= nrColumns - 1; counter++) {
				List<MasonGeometry> routeGeometries = routes.getGeometries();
				for (final MasonGeometry route : routeGeometries)
					if (route.hasAttribute("edgeIDs_" + counter))
						continue;
					else
						route.addAttribute("edgeIDs_" + counter, "None");
			}
		}
		ShapeFileExporter.write(directory, routes);
	}

	/**
	 * Forms route attributes and handles splitting long edgeIDs strings.
	 *
	 * @param masonGeometry The MasonGeometry object representing a route.
	 * @param routeData     The route data associated with the route.
	 */
	private static void formRouteAttributes(MasonGeometry masonGeometry, RouteData routeData) {
		String edgeIDs = ArrayUtils.toString(routeData.edgeIDsSequence);

		if (edgeIDs.length() <= FIELD_LIMIT) {
			masonGeometry.addAttribute("edgeIDs_0", edgeIDs);
		} else {
			String remainingEdges = edgeIDs;
			for (int counter = 0; remainingEdges.length() > 0; counter++) {
				if (counter >= nrColumns) {
					nrColumns += 1;
				}

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
