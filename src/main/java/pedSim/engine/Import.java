package pedSim.engine;

import java.io.File;
import java.io.InputStreamReader;
import java.io.Reader;
import java.net.URL;
import java.util.Arrays;
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
	String resourcePath;
	private static final Logger LOGGER = Logger.getLogger(Import.class.getName());

	/**
	 * Imports various data files required for the simulation based on the selected
	 * simulation parameters.
	 *
	 * @throws Exception If an error occurs during the import process.
	 */
	public void importFiles() throws Exception {
		resourcePath = Parameters.cityName;
		if (Parameters.javaProject)
			resourcePath = Parameters.localPath + resourcePath;
		System.out.println(Parameters.localPath);
		if (Parameters.cityName.equals("London")) {
			if (Parameters.testingLandmarks)
				resourcePath += "/landmarks";
			else if (Parameters.testingSubdivisions)
				resourcePath += "/subdivisions";
		}
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
		ClassLoader classLoader = getClass().getClassLoader();
		URL resourceURL = null;
		if (Parameters.javaProject)
			resourceURL = new File(resourcePath + "/tracks_distances.csv").toURI().toURL();
		else
			resourceURL = classLoader.getResource(resourcePath + "/tracks_distances.csv");
		Reader reader = new InputStreamReader(resourceURL.openStream());
		CSVReader readerDistances = new CSVReader(reader);
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

		ClassLoader classLoader = getClass().getClassLoader();
		URL urlShp = null;
		URL urlDbf = null;

		try {
			String[] strings = { "/edges", "/nodes", "/edgesDual", "/nodesDual" };
			VectorLayer[] vectorLayers = { PedSimCity.roads, PedSimCity.junctions, PedSimCity.intersectionsDual,
					PedSimCity.centroids };

			for (String string : strings) {
				String tmpPath = resourcePath + string;
				if (Parameters.javaProject) {
					urlShp = new File(tmpPath + ".shp").toURI().toURL();
					urlDbf = new File(tmpPath + ".dbf").toURI().toURL();
				} else {
					urlShp = classLoader.getResource(tmpPath + ".shp");
					urlDbf = classLoader.getResource(tmpPath + ".dbf");
				}
				System.out.println(tmpPath);
				VectorLayer.readShapefile(urlShp, urlDbf, vectorLayers[Arrays.asList(strings).indexOf(string)]);
			}
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

		ClassLoader classLoader = getClass().getClassLoader();
		URL urlShp = null;
		URL urlDbf = null;
		try {
			String[] strings = { "/landmarks", "/sight_lines2D" };
			VectorLayer[] vectorLayers = { PedSimCity.buildings, PedSimCity.sightLines };
			for (String string : strings) {
				String tmpPath = resourcePath + string;
				if (Parameters.javaProject) {
					urlShp = new File(tmpPath + ".shp").toURI().toURL();
					urlDbf = new File(tmpPath + ".dbf").toURI().toURL();
				} else {
					urlShp = classLoader.getResource(tmpPath + ".shp");
					urlDbf = classLoader.getResource(tmpPath + ".dbf");
				}
				VectorLayer.readShapefile(urlShp, urlDbf, vectorLayers[Arrays.asList(strings).indexOf(string)]);
			}
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

		URL urlShp = null;
		URL urlDbf = null;
		try {
			String tmpPath = resourcePath + "/barriers";
			if (Parameters.javaProject) {
				urlShp = new File(tmpPath + ".shp").toURI().toURL();
				urlDbf = new File(tmpPath + ".dbf").toURI().toURL();
			} else {
				ClassLoader classLoader = getClass().getClassLoader();
				urlShp = classLoader.getResource(tmpPath + ".shp");
				urlDbf = classLoader.getResource(tmpPath + ".dbf");
			}
			VectorLayer.readShapefile(urlShp, urlDbf, PedSimCity.barriers);
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
		LOGGER.info(layerName);
	}

	/**
	 * Imports empirical agent groups data for the simulation.
	 *
	 * @throws Exception If an error occurs during the import process.
	 */
	private void importEmpiricalGroups() throws Exception {

		ClassLoader classLoader = getClass().getClassLoader();
		URL resourceURL = null;
		if (Parameters.javaProject)
			resourceURL = new File(resourcePath + "/clusters.csv").toURI().toURL();
		else
			resourceURL = classLoader.getResource(resourcePath + "/clusters.csv");

		Reader reader = new InputStreamReader(resourceURL.openStream());
		CSVReader readerEmpiricalGroups = new CSVReader(reader);
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
