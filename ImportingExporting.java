package sim.app.geo.PedSimCity;

import java.io.FileReader;
import java.io.FileWriter;
import java.io.IOException;
import java.net.URL;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;

import org.apache.commons.lang3.ArrayUtils;

import com.opencsv.CSVReader;
import com.vividsolutions.jts.geom.Coordinate;
import com.vividsolutions.jts.geom.GeometryFactory;
import com.vividsolutions.jts.geom.LineString;

import sim.app.geo.UrbanSim.EdgeGraph;
import sim.app.geo.UrbanSim.NodeGraph;
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
			importDistances(inputDataDirectory);
		}
		else if (UserParameters.testingRegions) inputDataDirectory = "districtsData/"+UserParameters.cityName+"/";
		else inputDataDirectory = "data/"+UserParameters.cityName+"/";

		try {
			URL barriersFile = PedSimCity.class.getResource(inputDataDirectory+"/"+UserParameters.cityName+"_barriers.shp");
			ShapeFileImporter.read(barriersFile, PedSimCity.barriers);
			System.out.println("reading barriers layer");
			PedSimCity.barriers.generateGeometriesList();
		}
		catch (Exception e) {
			System.out.println("Barriers " + e + ". Do not worry unless you are testing barrier-based navigation ");
			PedSimCity.barriers = null;
		}
		try {
			System.out.println("reading buildings and sight lines layers");
			URL landmarksFile = PedSimCity.class.getResource(inputDataDirectory+"/"+UserParameters.cityName+"_landmarks.shp");
			URL sightLinesFile = PedSimCity.class.getResource(inputDataDirectory+"/"+UserParameters.cityName+"_sight_lines2D.shp");
			ShapeFileImporter.read(landmarksFile, PedSimCity.buildings);
			ShapeFileImporter.read(sightLinesFile, PedSimCity.sightLines);
			PedSimCity.buildings.generateGeometriesList();
			PedSimCity.sightLines.generateGeometriesList();
			PedSimCity.buildings.setID("buildingID");
		}
		catch (Exception e) {
			System.out.println("Landmarks " + e + ". Do not worry unless you are testing landmark-based navigation ");
			PedSimCity.buildings = null;
			PedSimCity.sightLines = null;
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

		if (UserParameters.empiricalABM) importGroups(inputDataDirectory);
		System.out.println("files imported successfully");
		UserParameters.setOutputFolder();
	}

	// save the simulation output
	public static void saveResults(int job) throws IOException {

		if (UserParameters.testingSpecificRoutes) {;} //densities are not analyzed in this case
		else {
			System.out.println("saving Densities");
			Bag edgesGeometries = PedSimCity.roads.getGeometries();
			String csvSegments = UserParameters.outputFolder+(job)+".csv";
			FileWriter writerDensitiesData = new FileWriter(csvSegments);
			int rGeoSize = edgesGeometries.size();

			List<String> rC = Arrays.asList(PedSimCity.routeChoiceModels);
			if (UserParameters.empiricalABM) {
				rC.clear();
				for (Group group : PedSimCity.groups) rC.add(group.groupName);
			}
			rC.add(0, "edgeID");
			CSVUtils.writeLine(writerDensitiesData, rC);

			for (int i = 0; i < rGeoSize; i++) {
				MasonGeometry segment = (MasonGeometry) edgesGeometries.objs[i];
				EdgeGraph ed = PedSimCity.edgesMap.get(segment.getIntegerAttribute("edgeID"));
				List<Integer> values = new ArrayList<Integer>(ed.densities.values());
				values.add(0, ed.getID());
				List<String> valuesString = new ArrayList<String>();
				for (int value : values) valuesString.add(Integer.toString(value));
				CSVUtils.writeLine(writerDensitiesData, valuesString);
			}
			writerDensitiesData.flush();
			writerDensitiesData.close();
		}

		System.out.println("saving Routes");
		VectorLayer routes = new VectorLayer();
		String	directory = UserParameters.outputFolderRoutes+"routes_"+(job);
		int columns = 0;
		for (RouteData rD : PedSimCity.routesData) {
			List<Integer> sequenceEdges = rD.sequenceEdges;
			List<Coordinate> allCoords = new ArrayList<Coordinate>();

			// creating the route geometry
			for (int i : sequenceEdges) {
				EdgeGraph edge = PedSimCity.edgesMap.get(i);
				LineString geometry = (LineString) edge.masonGeometry.geometry;
				Coordinate [] coords = geometry.getCoordinates();
				List<Coordinate> coordsCollection = Arrays.asList(coords);

				if (i == sequenceEdges.get(0)) {
					NodeGraph originNode = PedSimCity.nodesMap.get(rD.origin);
					if (!originNode.getCoordinate().equals(coords[0])) Collections.reverse(coordsCollection);
					allCoords.addAll(coordsCollection);
				}
				else {
					if (!coords[0].equals(allCoords.get(allCoords.size()-1))) Collections.reverse(coordsCollection);
					allCoords.addAll(coordsCollection);
				}

			}
			int limit = 254;
			GeometryFactory factory = new GeometryFactory();
			Coordinate[] allCoordsArray = new Coordinate[allCoords.size()];
			for (int i =0; i<= allCoords.size()-1; i ++) allCoordsArray[i] = allCoords.get(i);
			LineString lineGeometry = factory.createLineString(allCoordsArray);
			MasonGeometry mg  = new MasonGeometry(lineGeometry);

			mg.addIntegerAttribute("O", rD.origin);
			mg.addIntegerAttribute("D", rD.destination);
			if (UserParameters.empiricalABM) mg.addIntegerAttribute("group", rD.group);
			else mg.addStringAttribute("routeChoice", rD.routeChoice);

			//	the sequence of edgesIDs is truncated and split in different fields as a shapefile can handle max 254 characters per field
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

	public static void importDistances(String inputDataDirectory) throws IOException {
		/// read GPS trajectories distances
		System.out.println("reading distances");
		CSVReader readerDistances = new CSVReader(new FileReader(PedSimCity.class.getResource(inputDataDirectory).toString().substring(6)
				+"/"+UserParameters.cityName+"_tracks_distances.csv"));
		String[] nextLineDistances;

		int row = 0;
		while ((nextLineDistances = readerDistances.readNext()) != null) {
			row += 1;
			if (row == 1) continue; // header
			PedSimCity.distances.add(Float.parseFloat(nextLineDistances[2]));
		}
		readerDistances.close();
	}

	public static void importGroups(String inputDataDirectory) throws IOException {
		/// read GPS trajectories distances
		System.out.println("reading groups information");
		CSVReader readerGroups = new CSVReader(new FileReader(PedSimCity.class.getResource(inputDataDirectory).toString().substring(6)
				+"/"+UserParameters.cityName+"_groups.csv"));
		String[] nextLine;

		int row = 0;
		while ((nextLine = readerGroups.readNext()) != null) {
			row += 1;
			if (row == 1) continue;
			Group group = new Group();
			String groupName = "group"+Integer.toString(Integer.parseInt(nextLine[0]));
			group.setGroup(groupName, nextLine);
			PedSimCity.groups.add(group);
		}
		readerGroups.close();
	}
}
