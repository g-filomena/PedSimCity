package pedsimcity.main;

import java.io.FileReader;
import java.io.FileWriter;
import java.net.URL;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import org.apache.commons.lang3.ArrayUtils;

import com.opencsv.CSVReader;

import pedsimcity.agents.Group;
import pedsimcity.graph.EdgeGraph;
import pedsimcity.utilities.CSVUtils;
import pedsimcity.utilities.RouteData;
import pedsimcity.utilities.VectorLayer;
import sim.io.geo.ShapeFileExporter;
import sim.io.geo.ShapeFileImporter;
import sim.util.Bag;
import sim.util.geo.MasonGeometry;

public class ImportingExporting {

	public static void importFiles() throws Exception {

		String inputDataDirectory = null;
		if (UserParameters.testingLandmarks) {
			inputDataDirectory = "/pedsimcity/data/landmarksData" + "/" + UserParameters.cityName + "/";
			importDistances(inputDataDirectory);
		} else if (UserParameters.testingRegions)
			inputDataDirectory = "/pedsimcity/data/districtsData/" + UserParameters.cityName + "/";
		else
			inputDataDirectory = "/pedsimcity/data/" + UserParameters.cityName + "/";

		try {
			final URL barriersSHP = PedSimCity.class
					.getResource(inputDataDirectory + "/" + UserParameters.cityName + "_barriers.shp");
			final URL barriersDBF = PedSimCity.class
					.getResource(inputDataDirectory + "/" + UserParameters.cityName + "_barriers.dbf");

			ShapeFileImporter.read(barriersSHP, barriersDBF, PedSimCity.barriers);
			System.out.println("reading barriers layer");
			PedSimCity.barriers.generateGeometriesList();
		} catch (final Exception e) {
			System.out.println("Barriers " + e + ". Do not worry unless you are testing barrier-based navigation ");
			PedSimCity.barriers = null;
		}
		try {
			System.out.println("reading buildings and sight lines layers");
			final URL landmarksSHP = PedSimCity.class
					.getResource(inputDataDirectory + "/" + UserParameters.cityName + "_landmarks.shp");
			final URL landmarksDBF = PedSimCity.class
					.getResource(inputDataDirectory + "/" + UserParameters.cityName + "_landmarks.dbf");
			final URL sightLinesSHP = PedSimCity.class
					.getResource(inputDataDirectory + "/" + UserParameters.cityName + "_sight_lines2D.shp");
			final URL sightLinesDBF = PedSimCity.class
					.getResource(inputDataDirectory + "/" + UserParameters.cityName + "_sight_lines2D.dbf");
			ShapeFileImporter.read(landmarksSHP, landmarksDBF, PedSimCity.buildings);
			ShapeFileImporter.read(sightLinesSHP, sightLinesDBF, PedSimCity.sightLines);
			PedSimCity.buildings.generateGeometriesList();
			PedSimCity.sightLines.generateGeometriesList();
			PedSimCity.buildings.setID("buildingID");
		} catch (final Exception e) {
			System.out.println("Landmarks " + e + ". Do not worry unless you are testing landmark-based navigation ");
			PedSimCity.buildings = null;
			PedSimCity.sightLines = null;
		}

		// read the street network shapefiles and create the primal and the dual graph
		System.out.println("reading the graphs");
		final URL roadsSHP = PedSimCity.class
				.getResource(inputDataDirectory + "/" + UserParameters.cityName + "_edges.shp");
		final URL roadsDBF = PedSimCity.class
				.getResource(inputDataDirectory + "/" + UserParameters.cityName + "_edges.dbf");
		final URL junctionsSHP = PedSimCity.class
				.getResource(inputDataDirectory + "/" + UserParameters.cityName + "_nodes.shp");
		final URL junctionsDBF = PedSimCity.class
				.getResource(inputDataDirectory + "/" + UserParameters.cityName + "_nodes.dbf");
		final URL roadsDualSHP = PedSimCity.class
				.getResource(inputDataDirectory + "/" + UserParameters.cityName + "_edgesDual.shp");
		final URL roadsDualDBF = PedSimCity.class
				.getResource(inputDataDirectory + "/" + UserParameters.cityName + "_edgesDual.dbf");
		final URL centroidsSHP = PedSimCity.class
				.getResource(inputDataDirectory + "/" + UserParameters.cityName + "_nodesDual.shp");
		final URL centroidsDBF = PedSimCity.class
				.getResource(inputDataDirectory + "/" + UserParameters.cityName + "_nodesDual.dbf");

		ShapeFileImporter.read(roadsSHP, roadsDBF, PedSimCity.roads);
		ShapeFileImporter.read(junctionsSHP, junctionsDBF, PedSimCity.junctions);
		ShapeFileImporter.read(roadsDualSHP, roadsDualDBF, PedSimCity.intersectionsDual);
		ShapeFileImporter.read(centroidsSHP, centroidsDBF, PedSimCity.centroids);

		PedSimCity.network.fromGeomField(PedSimCity.junctions, PedSimCity.roads);
		PedSimCity.dualNetwork.fromGeomField(PedSimCity.centroids, PedSimCity.intersectionsDual);
		PedSimCity.roads.generateGeometriesList();
		PedSimCity.junctions.generateGeometriesList();
		PedSimCity.centroids.generateGeometriesList();

		if (UserParameters.empiricalABM)
			importGroups(inputDataDirectory);
		System.out.println("files imported successfully");
		UserParameters.setOutputFolder();
	}

	// save the simulation output
	public static void saveResults(int job) throws Exception {

		System.out.println("saving volumes");
		final Bag edgesGeometries = PedSimCity.roads.getGeometries();
		final String csvSegments = UserParameters.outputFolder + job + ".csv";
		final FileWriter writerVolumesData = new FileWriter(csvSegments);
		final int rGeoSize = edgesGeometries.size();

		ArrayList<String> header = new ArrayList<>();
		if (UserParameters.empiricalABM) {
			header = new ArrayList<>();
			for (final Group group : PedSimCity.groups)
				header.add(group.groupName);
		}
		else
			header = new ArrayList<>(Arrays.asList(PedSimCity.routeChoiceModels));
		header.add(0, "edgeID");
		CSVUtils.writeLine(writerVolumesData, header);

		for (int i = 0; i < rGeoSize; i++) {
			final MasonGeometry segment = (MasonGeometry) edgesGeometries.objs[i];
			final EdgeGraph ed = PedSimCity.edgesMap.get(segment.getIntegerAttribute("edgeID"));

			final List<String> row = new ArrayList<>();
			for (final String s : header) {
				if (s.equals("edgeID")) {
					row.add(0, Integer.toString(ed.getID()));
					continue;
				}
				row.add(Integer.toString(ed.volumes.get(s)));
			}
			CSVUtils.writeLine(writerVolumesData, row);
		}
		writerVolumesData.flush();
		writerVolumesData.close();

		System.out.println("saving Routes");
		final VectorLayer routes = new VectorLayer();
		final String directory = UserParameters.outputFolderRoutes + job;
		int columns = 0;

		for (final RouteData rD : PedSimCity.routesData) {

			final MasonGeometry mg = new MasonGeometry(rD.lineGeometry);

			mg.addIntegerAttribute("O", rD.origin);
			mg.addIntegerAttribute("D", rD.destination);
			if (UserParameters.empiricalABM) {
				mg.addStringAttribute("group", rD.group);
				mg.addStringAttribute("min", rD.onlyMinimisation);
				mg.addStringAttribute("heur", rD.localHeuristic);
				mg.addIntegerAttribute("regions", rD.regionBased);
				mg.addIntegerAttribute("routeMark", rD.routeMarks);
				mg.addIntegerAttribute("barrier", rD.barrierSubGoals);
				mg.addIntegerAttribute("distant", rD.distantLandmarks);
				mg.addDoubleAttribute("natural", rD.naturalBarriers);
				mg.addDoubleAttribute("severing", rD.severingBarriers);
			} else
				mg.addStringAttribute("routeChoice", rD.routeChoice);

			final int limit = 254;
			// the sequence of edgesIDs is truncated and split in different fields as a
			// shapefile can handle max 254 characters per field
			final String edgeIDs = ArrayUtils.toString(rD.sequenceEdges);
			String first_part = null;
			String other = null;
			if (edgeIDs.length() <= limit)
				mg.addAttribute("edgeIDs_0", edgeIDs);
			else {
				first_part = edgeIDs.substring(0, limit);
				mg.addAttribute("edgeIDs_0", first_part);
				other = edgeIDs.substring(limit);
				int counter = 1;
				while (true) {
					if (counter > columns)
						columns += 1;
					if (other.length() > limit) {
						first_part = other.substring(0, limit);
						mg.addAttribute("edgeIDs_" + counter, first_part);
						other = other.substring(limit);
						counter += 1;
					} else {
						mg.addAttribute("edgeIDs_" + counter, other);
						break;
					}
				}
			}
			routes.addGeometry(mg);
		}
		// this is to avoid to have geometries without the needed columns' values filled
		// in.
		if (columns > 0) {
			routes.generateGeometriesList();
			for (int column = 1; column <= columns; column++) {
				final ArrayList<MasonGeometry> routeGeometries = routes.geometriesList;
				for (final MasonGeometry route : routeGeometries)
					if (route.hasAttribute("edgeIDs_" + column))
						continue;
					else
						route.addAttribute("edgeIDs_" + column, "None");
			}
		}

		ShapeFileExporter.write(directory, routes);
		PedSimCity.routesData = new ArrayList<>();
	}

	public static void importDistances(String inputDataDirectory) throws Exception {
		/// read GPS trajectories distances
		System.out.println("reading distances");
		final CSVReader readerDistances = new CSVReader(
				new FileReader(PedSimCity.class.getResource(inputDataDirectory).toString().substring(6) + "/"
						+ UserParameters.cityName + "_tracks_distances.csv"));
		String[] nextLineDistances;

		int row = 0;
		while ((nextLineDistances = readerDistances.readNext()) != null) {
			row += 1;
			if (row == 1)
				continue; // header
			PedSimCity.distances.add(Float.parseFloat(nextLineDistances[2]));
		}
		readerDistances.close();
	}

	public static void importGroups(String inputDataDirectory) throws Exception {
		/// read GPS trajectories distances
		System.out.println("reading groups information");
		final CSVReader readerGroups = new CSVReader(
				new FileReader(PedSimCity.class.getResource(inputDataDirectory).toString().substring(6) + "/"
						+ UserParameters.cityName + "_clusters.csv"));
		String[] nextLine;

		int row = 0;
		while ((nextLine = readerGroups.readNext()) != null) {
			row += 1;
			if (row == 1)
				continue;
			final Group group = new Group();
			final String groupName = nextLine[0];
			group.setGroup(groupName, nextLine);
			PedSimCity.groups.add(group);
		}
		readerGroups.close();
	}
}
