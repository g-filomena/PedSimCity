package pedsim.cityimage.engine;

import com.opencsv.CSVReader;
import java.io.InputStreamReader;
import java.io.Reader;
import java.net.URL;
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
    // The test design is already resolved by CityImageSimulationModule.applyDefaults(), which runs
    // before this. Do not call TestPars.defineMode() here: Engine.runJobs reaches this method after
    // the command line has been applied, so it would reinstate the design's own defaults for
    // numberTripsPerAgent and jobs and discard whatever was asked for.

    if (TestPars.testingLandmarks) {
      importDistances();
      readBuildings();
      readSightLines();
    } else if (TestPars.testingSubdivisions) readBarriers();
    else if (TestPars.testingModels) {
      readBuildings();
      readSightLines();
      readBarriers();
    }
    if (!TestPars.routeLengths.isBlank()) {
      importRouteLengths();
    }
    // Read the street network and build the graph
    readGraphs();
  }

  /**
   * Reads the observed walked route lengths named by {@link TestPars#routeLengths}: a header, then
   * {@code metres,weight} rows. Rows with a non-positive length or weight are refused.
   *
   * @throws Exception If the file is missing or cannot be read.
   */
  private void importRouteLengths() throws Exception {
    String resourceName =
        Pars.cityName + "/" + Pars.cityName + "_" + TestPars.routeLengths + ".csv";
    URL fileUrl = CLASSLOADER.getResource(resourceName);
    if (fileUrl == null) {
      throw new IllegalStateException("Resource not found: " + resourceName);
    }
    TestPars.routeLengthMetres.clear();
    TestPars.routeLengthWeights.clear();
    try (CSVReader reader = new CSVReader(new InputStreamReader(fileUrl.openStream()))) {
      String[] line;
      boolean header = true;
      while ((line = reader.readNext()) != null) {
        if (header) {
          header = false;
          continue;
        }
        double metres = Double.parseDouble(line[0]);
        double weight = Double.parseDouble(line[1]);
        if (!(metres > 0.0) || !(weight > 0.0)) {
          throw new IllegalStateException(
              resourceName + ": length and weight must be positive, got " + String.join(",", line));
        }
        TestPars.routeLengthMetres.add(metres);
        TestPars.routeLengthWeights.add(weight);
      }
    }
    if (TestPars.routeLengthMetres.isEmpty()) {
      throw new IllegalStateException(resourceName + " holds no route lengths");
    }
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
      if (row == 1) continue; // Skip header
      TestPars.distances.add(Float.parseFloat(nextLineDistances[2]));
    }
    readerDistances.close();
  }
}
