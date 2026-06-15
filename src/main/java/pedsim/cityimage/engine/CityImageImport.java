package pedsim.cityimage.engine;

import java.io.InputStreamReader;
import java.io.Reader;
import java.net.URL;

import com.opencsv.CSVReader;

import pedsim.cityimage.parameters.TestPars;
import pedsim.core.parameters.Pars;

/**
 * This class is responsible for importing various data files required for the
 * simulation based on the selected simulation parameters. It includes methods
 * for importing distances, barriers, landmarks and sight lines, road network
 * graphs, and empirical agent groups data.
 */
public class CityImageImport extends pedsim.core.engine.Import {

	/**
	 * Imports various data files required for the simulation based on the selected
	 * simulation parameters.
	 *
	 * @throws Exception If an error occurs during the import process.
	 */
	@Override
	public void importFiles() throws Exception {

		if (TestPars.testingLandmarks) {
			importDistances();
			readLandmarksAndSightLines();
		} else if (TestPars.testingSubdivisions)
			readBarriers();
		else if (TestPars.testingModels) {
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
		String resourceName = Pars.cityName + "/" + Pars.cityName + "_distances.csv";
		URL fileUrl = CLASSLOADER.getResource(resourceName);
		if (fileUrl == null) {
			throw new IllegalStateException("Resource not found: " + resourceName);
		}

		Reader reader = new InputStreamReader(fileUrl.openStream());
		CSVReader readerDistances = new CSVReader(reader);
		String[] nextLineDistances;

		int row = 0;
		while ((nextLineDistances = readerDistances.readNext()) != null) {
			row += 1;
			if (row == 1)
				continue; // Skip header
			TestPars.distances.add(Float.parseFloat(nextLineDistances[2]));
		}
		readerDistances.close();
	}
}
