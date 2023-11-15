package pedSim.engine;

import java.io.File;
import java.io.FileReader;
import java.net.URL;
import java.util.logging.Logger;

import com.opencsv.CSVReader;

import pedSim.agents.EmpiricalAgentsGroup;
import pedSim.utilities.StringEnum.Groups;
import sim.field.geo.VectorLayer;

/**
 * This class is responsible for importing various data files required for the
 * simulation based on the selected simulation parameters. It includes methods
 * for importing distances, barriers, landmarks and sight lines, road network
 * graphs, and empirical agent groups data.
 */
public class Import {

	/**
	 * The base data directory path for the simulation data files.
	 */
	String dataDirectory = getClass().getClassLoader().toString();

//	public static String dataDirectory = "C:\\Users\\gfilo\\OneDrive - The University of Liverpool\\Scripts\\pedsimcityJava\\pedSim\\pedSim";
	private static final Logger LOGGER = Logger.getLogger(Import.class.getName());

	/**
	 * Imports various data files required for the simulation based on the selected
	 * simulation parameters.
	 *
	 * @throws Exception If an error occurs during the import process.
	 */
	public void importFiles() throws Exception {

		// Determine the input data directory based on the simulation parameters
		ClassLoader classLoader = getClass().getClassLoader();
		String resourcePath = Parameters.cityName + File.separator;
		if (Parameters.javaProject)
			resourcePath = "C:\\Users\\gfilo\\OneDrive - The University of Liverpool\\Scripts\\pedsimcity\\src\\main\\resources\\"
					+ resourcePath;
		if (Parameters.cityName.equals("London")) {
			if (Parameters.testingLandmarks)
				resourcePath += "landmarks";
			else if (Parameters.testingSubdivisions)
				resourcePath += "subdivisions";
		}

		URL resourceURL = null;
		if (Parameters.javaProject)
			dataDirectory = resourcePath;
		else {
			resourceURL = classLoader.getResource(resourcePath);
			dataDirectory = resourceURL.getPath();
		}

		System.out.println(dataDirectory);
		if (Parameters.testingLandmarks) {
			importDistances();
			readLandmarksAndSightLines();
		} else if (Parameters.testingSubdivisions)
			readBarriers();
		else if (Parameters.empirical) {
			readLandmarksAndSightLines();
			readBarriers();
			importEmpiricalGroups();
		} else if (Parameters.testingModels) {
			readLandmarksAndSightLines();
			readBarriers();
		}
		// Read the street network shapefiles and create the primal and the dual graph
		readGraphs();
	}

	/**
	 * Imports GPS trajectory-derived distances required for the simulation.
	 *
	 * @throws Exception If an error occurs during the import process.
	 */
	private void importDistances() throws Exception {
		// Read GPS trajectories distances
		String filePath = dataDirectory + File.separator + "tracks_distances.csv";
		final CSVReader readerDistances = new CSVReader(new FileReader(filePath));
		String[] nextLineDistances;

		int row = 0;
		while ((nextLineDistances = readerDistances.readNext()) != null) {
			row += 1;
			if (row == 1)
				continue; // Skip header
			PedSimCity.distances.add(Float.parseFloat(nextLineDistances[2]));
		}
		readerDistances.close();
	}

	/**
	 * Reads and imports road network graphs required for the simulation.
	 *
	 * @throws Exception If an error occurs during the import process.
	 */
	private void readGraphs() throws Exception {
		try {
			VectorLayer.readShapefile(dataDirectory + File.separator + "edges", PedSimCity.roads);
			VectorLayer.readShapefile(dataDirectory + File.separator + "nodes", PedSimCity.junctions);
			VectorLayer.readShapefile(dataDirectory + File.separator + "edgesDual", PedSimCity.intersectionsDual);
			VectorLayer.readShapefile(dataDirectory + File.separator + "nodesDual", PedSimCity.centroids);
			PedSimCity.network.fromStreetJunctionsSegments(PedSimCity.junctions, PedSimCity.roads);
			PedSimCity.dualNetwork.fromStreetJunctionsSegments(PedSimCity.centroids, PedSimCity.intersectionsDual);
			LOGGER.info("Graphs successfully imported.");
		} catch (Exception e) {
			handleImportError("Importing Graphs failed", e);
		}
	}

	/**
	 * Reads and imports landmarks and sight lines data for the simulation.
	 *
	 * @throws Exception If an error occurs during the import process.
	 */
	private void readLandmarksAndSightLines() throws Exception {
		try {
			VectorLayer.readShapefile(dataDirectory + File.separator + "landmarks", PedSimCity.buildings);
			VectorLayer.readShapefile(dataDirectory + File.separator + "sight_lines2D", PedSimCity.sightLines);
			PedSimCity.buildings.setID("buildingID");
			LOGGER.info("Landmarks successfully imported.");
		} catch (Exception e) {
			handleImportError("Importing Landmarks Failed", e);
		}
	}

	/**
	 * Reads and imports barriers data for the simulation.
	 *
	 * @throws Exception If an error occurs during the import process.
	 */
	private void readBarriers() throws Exception {
		try {
			VectorLayer.readShapefile(dataDirectory + File.separator + "barriers", PedSimCity.barriers);
			LOGGER.info("Barriers successfully imported.");
		} catch (Exception e) {
			handleImportError("Importing Barriers Failed", e);
		}
	}

	/**
	 * Imports empirical agent groups data for the simulation.
	 *
	 * @throws Exception If an error occurs during the import process.
	 */
	private static void handleImportError(String layerName, Exception e) {
		// Perform additional error handling or logging as needed
	}

	/**
	 * Imports empirical agent groups data for the simulation.
	 *
	 * @throws Exception If an error occurs during the import process.
	 */
	private void importEmpiricalGroups() throws Exception {

		String filePath = dataDirectory + File.separator + "clusters.csv";
		final CSVReader readerEmpiricalGroups = new CSVReader(new FileReader(filePath));
		String[] nextLine;

		int row = 0;
		while ((nextLine = readerEmpiricalGroups.readNext()) != null) {
			row += 1;
			if (row == 1)
				continue;
			final EmpiricalAgentsGroup empiricalGroup = new EmpiricalAgentsGroup();
			final String groupName = nextLine[0];
			empiricalGroup.setGroup(Groups.valueOf(groupName), nextLine);
			PedSimCity.empiricalGroups.add(empiricalGroup);
		}
		readerEmpiricalGroups.close();
	}
}
