package sim.app.geo.PedSimCity;

import java.io.FileReader;
import java.io.FileWriter;
import java.io.IOException;
import java.net.URL;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collection;
import java.util.List;

import org.apache.commons.lang3.ArrayUtils;

import com.opencsv.CSVReader;
import com.vividsolutions.jts.geom.LineString;
import com.vividsolutions.jts.operation.linemerge.LineMerger;

import sim.app.geo.UrbanSim.EdgeGraph;
import sim.app.geo.UrbanSim.VectorLayer;
import sim.io.geo.ShapeFileExporter;
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
		else if (UserParameters.testingSpecificRoutes) inputDataDirectory = "data/"+UserParameters.cityName+"/";
		if (UserParameters.testingLandmarks) {
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

		if (UserParameters.testingRegions) {

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
		UserParameters.setOutputFolder();
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

		VectorLayer routes = new VectorLayer();
		String	directory = UserParameters.outputFolderRoutes+"_routes_"+(job);
		int columns = 0;
		for (RouteData rD : PedSimCity.routesData) {

			List<Integer> sequenceEdges = rD.sequenceEdges;
			LineMerger merger = new LineMerger();

			for (int i : sequenceEdges) {
				EdgeGraph edge = PedSimCity.edgesMap.get(i);
				LineString geometry = (LineString) edge.masonGeometry.geometry;
				merger.add(geometry);
			}
			int limit = 254;
			Collection<LineString> collection = merger.getMergedLineStrings();
			LineString mergedLine = collection.iterator().next();
			MasonGeometry mg = new MasonGeometry(mergedLine);
			mg.addIntegerAttribute("O", rD.origin);
			mg.addIntegerAttribute("D", rD.destination);
			mg.addStringAttribute("routeChoice", rD.routeChoice);

			// the sequence of edgesIDs is truncated and split in different fields as a shapefile can handle max 254 characters per field
			String edgeIDs = ArrayUtils.toString(rD.sequenceEdges);
			String first_part = null;
			String other = null;
			if (edgeIDs.length() <= limit) mg.addAttribute("edgesID_0", edgeIDs);
			else {
				first_part = edgeIDs.substring(0, limit);
				mg.addAttribute("edgesID_0", first_part);
				other = edgeIDs.substring(limit);
				int counter = 1;
				while (true) {
					if (counter > columns) columns += 1;
					if (other.length() > limit) {
						first_part = other.substring(0, limit);
						mg.addAttribute("edgesID_"+counter, first_part);
						other = other.substring(limit);
						counter += 1;
					}
					else {
						mg.addAttribute("edgesID_"+counter, other);
						break;
					}
				}
			}
			routes.addGeometry(mg);
		}
		// this is to avoid to have geometries without the needed columns' values filled in.
		if (columns > 0) {
			routes.generateGeometriesList();
			for (int column = 1; column<= columns; column++)
			{
				ArrayList<MasonGeometry> routeGeometries = routes.geometriesList;
				for (MasonGeometry route : routeGeometries)
				{
					if (route.hasAttribute("edgesID_"+column)) continue;
					else route.addAttribute("edgesID_"+column, "None");

				}
			}
		}

		ShapeFileExporter.write(directory, routes);
		PedSimCity.routesData.clear();
	}

}
