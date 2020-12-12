package sim.app.geo.PedSimCity;

import java.io.FileWriter;
import java.io.IOException;
import java.net.URL;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;

import org.apache.commons.lang3.ArrayUtils;

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
		if (UserParameters.testingRegions) {
			inputDataDirectory = "districtsData/"+UserParameters.cityName+"/";
			System.out.println("reading barriers layer");
			URL barriersFile = PedSimCity.class.getResource(inputDataDirectory+"/"+UserParameters.cityName+"_barriers.shp");
			ShapeFileImporter.read(barriersFile, PedSimCity.barriers);
			PedSimCity.barriers.generateGeometriesList();
		}
		else if (UserParameters.testingSpecificRoutes) inputDataDirectory = "data/"+UserParameters.cityName+"/";

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

	// save the simulation output
	public static void saveResults(int job) throws IOException {

		System.out.println("saving Densities");
		Bag edgesGeometries = PedSimCity.roads.getGeometries();
		String csvSegments = null;

		if (UserParameters.testingRegions) {
			csvSegments = UserParameters.outputFolder+(job)+".csv";
			FileWriter writerDensitiesData = new FileWriter(csvSegments);
			CSVUtils.writeLine(writerDensitiesData, Arrays.asList("edgeID", "AC", "RB", "BB", "RBB"));


			int rGeoSize = edgesGeometries.size();
			for (int i = 0; i < rGeoSize; i++) {
				MasonGeometry segment = (MasonGeometry) edgesGeometries.objs[i];
				EdgeGraph ed = PedSimCity.edgesMap.get(segment.getIntegerAttribute("edgeID"));
				CSVUtils.writeLine(writerDensitiesData, Arrays.asList(Integer.toString(
						ed.getID()), Integer.toString(ed.AC), Integer.toString(ed.RB), Integer.toString(ed.BB),	Integer.toString(ed.RBB)));
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
			mg.addStringAttribute("routeChoice", rD.routeChoice);

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

}
