package pedSim.engine;

import java.io.File;
import java.io.FileReader;
import java.util.logging.Logger;

import com.opencsv.CSVReader;

import pedSim.agents.EmpiricalAgentsGroup;
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

		if (Parameters.cityName.equals("London")) {
			if (Parameters.testingLandmarks)
				resourcePath += "landmarks";
			else if (Parameters.testingSubdivisions)
				resourcePath += "subdivisions";
		}
		resourcePath = "C:\\Users\\gfilo\\OneDrive - The University of Liverpool\\Scripts\\pedsimcity\\src\\main\\resources\\"
				+ resourcePath;
		System.out.println(resourcePath);
//		URL resourceURL = classLoader.getResource(resourcePath);
//		dataDirectory = resourceURL.getPath();
		dataDirectory = resourcePath;
		System.out.println(dataDirectory);

		if (Parameters.testingLandmarks) {
				try {
					importDistances();
					readLandmarksAndSightLines();
					LOGGER.info("Landmarks and Sight Lines successfully imported.");
				} catch (Exception e) {
					handleImportError("landmarks and sight lines", e);
				}
		}
		else if (Parameters.testingSubdivisions) {
			// Import various data files
			try {
				readBarriers();
				LOGGER.info("Barriers successfully imported.");
			} catch (Exception e) {
				handleImportError("Reading Barriers", e);
			}
		}
		else if (Parameters.empirical) {
			// Import various data files
			try {
				readBarriers();
				readLandmarksAndSightLines();
				importEmpiricalGroups();
				LOGGER.info("Data successfully imported.");
			} catch (Exception e) {
				handleImportError("Reading Barriers", e);
			}
		}

		// Read the street network shapefiles and create the primal and the dual graph
		try {
			readGraphs();
			LOGGER.info("Graphs successfully imported.");
		} catch (Exception e) {
			handleImportError("graphs", e);
			PedSimCity.roads = null;
			PedSimCity.junctions = null;
			PedSimCity.intersectionsDual = null;
			PedSimCity.centroids = null;
		}
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
	 * Reads and imports barriers data for the simulation.
	 *
	 * @throws Exception If an error occurs during the import process.
	 */
	private void readBarriers() throws Exception {
		VectorLayer.readShapefile(dataDirectory + File.separator + "barriers", PedSimCity.barriers);
	}

	/**
	 * Reads and imports landmarks and sight lines data for the simulation.
	 *
	 * @throws Exception If an error occurs during the import process.
	 */
	private void readLandmarksAndSightLines() throws Exception {
		VectorLayer.readShapefile(dataDirectory + File.separator + "landmarks", PedSimCity.buildings);
		VectorLayer.readShapefile(dataDirectory + File.separator + "sight_lines2D", PedSimCity.sightLines);
		PedSimCity.buildings.setID("buildingID");
	}

	/**
	 * Reads and imports road network graphs required for the simulation.
	 *
	 * @throws Exception If an error occurs during the import process.
	 */
	private void readGraphs() throws Exception {

		VectorLayer.readShapefile(dataDirectory + File.separator + "edges", PedSimCity.roads);
		VectorLayer.readShapefile(dataDirectory + File.separator + "nodes", PedSimCity.junctions);
		VectorLayer.readShapefile(dataDirectory + File.separator + "edgesDual", PedSimCity.intersectionsDual);
		VectorLayer.readShapefile(dataDirectory + File.separator + "nodesDual", PedSimCity.centroids);

		PedSimCity.network.fromStreetJunctionsSegments(PedSimCity.junctions, PedSimCity.roads);
		PedSimCity.dualNetwork.fromStreetJunctionsSegments(PedSimCity.centroids, PedSimCity.intersectionsDual);
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
			empiricalGroup.setGroup(groupName, nextLine);
			PedSimCity.empiricalGroups.add(empiricalGroup);
		}
		readerEmpiricalGroups.close();
	}
}
