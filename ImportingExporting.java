package sim.app.geo.pedSimCity;

import java.io.FileReader;
import java.io.FileWriter;
import java.io.IOException;
import java.net.URL;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import org.apache.commons.lang3.ArrayUtils;
import org.supercsv.io.CsvMapWriter;
import org.supercsv.io.ICsvMapWriter;
import org.supercsv.prefs.CsvPreference;

import com.opencsv.CSVReader;

import sim.app.geo.urbanSim.EdgeGraph;
import sim.io.geo.ShapeFileImporter;
import sim.util.Bag;
import sim.util.geo.MasonGeometry;

public class ImportingExporting {


	public static void importFiles() throws IOException {

		String inputDataDirectory = null;
		if (UserParameters.testingLandmarks) {
			inputDataDirectory = "landmarksData"+"/"+UserParameters.cityName+"/";
			/// read distances

			if (!UserParameters.testingSpecificRoutes) {
				System.out.println("reading distances");
				CSVReader readerDistances = new CSVReader(new FileReader(PedSimCity.class.getResource(inputDataDirectory).toString().substring(6)
						+"/"+UserParameters.cityName+"_tracks_distances.csv"));
				String[] nextLineDistances;

				int ds = 0;
				while ((nextLineDistances = readerDistances.readNext()) != null) {
					ds += 1;
					if (ds == 1) continue;
					PedSimCity.distances.add(Float.parseFloat(nextLineDistances[2]));
				}
				readerDistances.close();
			}
		}
		else if (UserParameters.testingRegions) inputDataDirectory = "districtsData/"+UserParameters.cityName+"/";
		else if (UserParameters.fiveElements) inputDataDirectory = "data/"+UserParameters.cityName+"/";

		if (UserParameters.testingLandmarks  || UserParameters.fiveElements) {
			// read buildings
			System.out.println("reading buildings and sight lines layers");
			URL landmarksFile = PedSimCity.class.getResource(inputDataDirectory+"/"+UserParameters.cityName+"_landmarks.shp");
			URL sightLinesFile = PedSimCity.class.getResource(inputDataDirectory+"/"+UserParameters.cityName+"_sight_lines2D.shp");
			ShapeFileImporter.read(landmarksFile, PedSimCity.buildings);
			ShapeFileImporter.read(sightLinesFile, PedSimCity.sightLines);
			PedSimCity.buildings.generateGeometriesList();
			PedSimCity.sightLines.generateGeometriesList();
			PedSimCity.buildings.setID("buildingID");
		}

		if (UserParameters.testingRegions || UserParameters.fiveElements) {

			System.out.println("reading barriers layer");
			URL barriersFile = PedSimCity.class.getResource(inputDataDirectory+"/"+UserParameters.cityName+"_barriers.shp");
			ShapeFileImporter.read(barriersFile, PedSimCity.barriers);
			PedSimCity.barriers.generateGeometriesList();
		}

		// read the street network shapefiles and create the primal and the dual graph
		System.out.println("reading the graphs");

		URL roadsFile = PedSimCity.class.getResource(inputDataDirectory+"/"+UserParameters.cityName+"_edges.shp");
		URL junctionsFile = PedSimCity.class.getResource(inputDataDirectory+"/"+UserParameters.cityName+"_nodes.shp");
		URL roadsDualFile = PedSimCity.class.getResource(inputDataDirectory+"/"+UserParameters.cityName+"_edgesDual.shp");
		URL centroidsFile = PedSimCity.class.getResource(inputDataDirectory+"/"+UserParameters.cityName+"_nodesDual.shp");

		ShapeFileImporter.read(roadsFile, PedSimCity.roads);
		ShapeFileImporter.read(junctionsFile, PedSimCity.junctions);
		ShapeFileImporter.read(roadsDualFile, PedSimCity.intersectionsDual);
		ShapeFileImporter.read(centroidsFile, PedSimCity.centroids);

		PedSimCity.network.fromGeomField(PedSimCity.junctions, PedSimCity.roads);
		PedSimCity.dualNetwork.fromGeomField(PedSimCity.centroids, PedSimCity.intersectionsDual);
		PedSimCity.roads.generateGeometriesList();
		PedSimCity.junctions.generateGeometriesList();
		PedSimCity.centroids.generateGeometriesList();

		System.out.println("files imported successfully");
	}



	// it reads OD from an existing file
	public static void readingOD(String inputFile) throws IOException {

		CSVReader readerOD;
		readerOD = new CSVReader(new FileReader(inputFile));
		String[] nextLine;
		UserParameters.OR.clear();
		UserParameters.DE.clear();

		int dv = 0;
		while ((nextLine = readerOD.readNext()) != null) {
			dv += 1;
			if (dv == 1) continue;
			UserParameters.OR.add(Integer.parseInt(nextLine[1]));
			UserParameters.DE.add(Integer.parseInt(nextLine[2]));
		}
		readerOD.close();
	}


	// save the simulation output
	public static void saveCSV(int job) throws IOException {

		System.out.println("saving Densities");
		Bag edgesGeometries = PedSimCity.roads.getGeometries();
		String csvSegments = null;

		if (UserParameters.testingRegions) {
			csvSegments = UserParameters.outputFolder+(job)+".csv";
			FileWriter writerDensitiesData = new FileWriter(csvSegments);
			CSVUtils.writeLine(writerDensitiesData, Arrays.asList("edgeID", "AC", "RB", "BB", "BRB"));


			int rGeoSize = edgesGeometries.size();
			for (int i = 0; i < rGeoSize; i++) {
				MasonGeometry segment = (MasonGeometry) edgesGeometries.objs[i];
				EdgeGraph ed = PedSimCity.edgesMap.get(segment.getIntegerAttribute("edgeID"));
				CSVUtils.writeLine(writerDensitiesData, Arrays.asList(Integer.toString(
						ed.getID()), Integer.toString(ed.roadDistance),
						Integer.toString(ed.angularChange), Integer.toString(ed.angularChangeRegions),
						Integer.toString(ed.angularChangeBarriers),	Integer.toString(ed.angularChangeRegionsBarriers)));
			}
			writerDensitiesData.flush();
			writerDensitiesData.close();
		}

		else if (UserParameters.testingLandmarks) {

			csvSegments = UserParameters.outputFolder+(job)+".csv";
			FileWriter writerDensitiesData = new FileWriter(csvSegments);
			CSVUtils.writeLine(writerDensitiesData, Arrays.asList("edgeID", "RD", "AC", "RL", "AL", "LL", "GL"));

			int rGeoSize = edgesGeometries.size();
			for (int i = 0; i < rGeoSize; i++) {
				MasonGeometry segment = (MasonGeometry) edgesGeometries.objs[i];
				EdgeGraph ed = PedSimCity.edgesMap.get(segment.getIntegerAttribute("edgeID"));
				CSVUtils.writeLine(writerDensitiesData, Arrays.asList(Integer.toString(ed.getID()),
						Integer.toString(ed.roadDistance), Integer.toString(ed.angularChange),
						Integer.toString(ed.roadDistanceLandmarks), Integer.toString(ed.angularChangeLandmarks),
						Integer.toString(ed.localLandmarks), Integer.toString(ed.globalLandmarks)));
			}
			writerDensitiesData.flush();
			writerDensitiesData.close();
		}

		System.out.println("saving Routes");

		for (String cr : PedSimCity.criteria) {

			String	csvRoutes = UserParameters.outputFolder+cr+"_routes_"+(job)+".csv";

			List<String> header = new ArrayList<String>();
			header.addAll(Arrays.asList(new String[] {"routeID", "origin", "destination"}));

			for (int i = 0; i < edgesGeometries.size(); i++) {
				MasonGeometry segment = (MasonGeometry) edgesGeometries.objs[i];
				header.add(Integer.toString(segment.getIntegerAttribute("edgeID")));
			}

			String[] headerArray = new String[header.size()];
			header.toArray(headerArray);
			List<Map<String, Object>> rows = new ArrayList<Map<String, Object>>();

			for (RouteData rD : PedSimCity.routesData) {
				String routeCriteria =  rD.criteria;
				if (cr != routeCriteria) continue;
				Map<String, Object> row = new HashMap<String, Object>();
				List<Integer> sequenceEdges = rD.sequenceEdges;

				int originNode =  rD.origin;
				int destinationNode = rD.destination;
				row.put(headerArray[0], Integer.toString(originNode)+"_"+Integer.toString(destinationNode));
				row.put(headerArray[1], originNode);
				row.put(headerArray[2], destinationNode);

				for (int e = 0; e < sequenceEdges.size(); e++) {
					Integer position = ArrayUtils.indexOf(headerArray, Integer.toString(sequenceEdges.get(e)));
					row.put(headerArray[position], 1);
				}
				rows.add(row);
			}

			ICsvMapWriter mapWriter = null;
			try {
				mapWriter = new CsvMapWriter(new FileWriter(csvRoutes), CsvPreference.STANDARD_PREFERENCE);
				// write the header
				mapWriter.writeHeader(headerArray);
				// write the customer maps
				for (int r = 0; r < rows.size(); r++) mapWriter.write(rows.get(r), headerArray);
			}
			finally {if( mapWriter != null ) mapWriter.close();}
		}
		PedSimCity.routesData.clear();
	}


}
