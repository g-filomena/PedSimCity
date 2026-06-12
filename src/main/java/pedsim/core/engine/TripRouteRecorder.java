package pedsim.core.engine;

import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.ConcurrentLinkedQueue;
import java.util.logging.Logger;
import org.locationtech.jts.geom.Coordinate;
import org.locationtech.jts.geom.LineString;
import org.locationtech.jts.planargraph.DirectedEdge;
import pedsim.core.utilities.LoggerUtil;
import sim.graph.EdgeGraph;
import sim.graph.NodeGraph;

/**
 * Records the exact sequence of node and edge IDs traversed by each agent
 * during completed journeys and exports them to a file upon simulation completion.
 */
public class TripRouteRecorder {
  private static final Logger logger = LoggerUtil.getLogger();

  public static class TripRecord {
    public int agentId;
    public double startStep;
    public double endStep;
    public boolean vulnerable;
    public int originNodeId;
    public int destNodeId;
    public List<Coordinate> pathCoords = new ArrayList<>();
    public List<Integer> edgeIds = new ArrayList<>();
    public List<Integer> nodeIds = new ArrayList<>();
    public List<Coordinate> spookLocations = new ArrayList<>();
  }

  private static final ConcurrentLinkedQueue<TripRecord> records = new ConcurrentLinkedQueue<>();

  public static void recordTrip(pedsim.core.agents.Agent agent, double startStep, double endStep, List<DirectedEdge> edges, boolean vulnerable) {
    if (edges == null || edges.isEmpty()) {
      return;
    }
    TripRecord record = new TripRecord();
    record.agentId = agent.agentID;
    record.startStep = startStep;
    record.endStep = endStep;
    record.vulnerable = vulnerable;
    record.spookLocations = new ArrayList<>(agent.spookLocations);
    
    // First node of the first edge is the origin node
    DirectedEdge firstEdge = edges.get(0);
    NodeGraph startNode = (NodeGraph) firstEdge.getFromNode();
    record.originNodeId = startNode.getID();
    record.nodeIds.add(startNode.getID());

    List<Coordinate> coords = new ArrayList<>();
    for (DirectedEdge de : edges) {
      EdgeGraph edge = (EdgeGraph) de.getEdge();
      record.edgeIds.add(edge.getID());
      NodeGraph toNode = (NodeGraph) de.getToNode();
      record.nodeIds.add(toNode.getID());

      LineString line = edge.getLine();
      Coordinate[] cArr = line.getCoordinates();
      Coordinate fromCoord = ((NodeGraph) de.getFromNode()).getCoordinate();
      
      boolean reverse = fromCoord.distance(cArr[0]) > fromCoord.distance(cArr[cArr.length - 1]);
      if (reverse) {
        for (int j = cArr.length - 1; j >= 0; j--) {
          Coordinate c = cArr[j];
          addCoordinate(coords, c);
        }
      } else {
        for (int j = 0; j < cArr.length; j++) {
          Coordinate c = cArr[j];
          addCoordinate(coords, c);
        }
      }
    }
    
    record.pathCoords = coords;
    record.destNodeId = record.nodeIds.get(record.nodeIds.size() - 1);
    records.add(record);

    double distance = 0;
    if (coords.size() >= 2) {
      for (int i = 0; i < coords.size() - 1; i++) {
        Coordinate a = coords.get(i);
        Coordinate b = coords.get(i + 1);
        double dx = b.x - a.x;
        double dy = b.y - a.y;
        distance += Math.sqrt(dx * dx + dy * dy);
      }
    }
    SimulationStateStore.getInstance().addCompletedTrip(vulnerable, distance);
  }

  private static void addCoordinate(List<Coordinate> coords, Coordinate c) {
    if (coords.isEmpty()) {
      coords.add(c);
    } else {
      Coordinate last = coords.get(coords.size() - 1);
      if (Math.abs(last.x - c.x) > 1e-6 || Math.abs(last.y - c.y) > 1e-6) {
        coords.add(c);
      }
    }
  }

  public static void clear() {
    records.clear();
  }

  public static List<TripRecord> getRecords() {
    return new ArrayList<>(records);
  }

  public static void saveToFile(String filename) {
    logger.info("[TripRouteRecorder] Saving " + records.size() + " trips to " + filename);
    try (FileWriter writer = new FileWriter(filename)) {
      writer.write("agent_id,start_step,end_step,origin_node_id,destination_node_id,edge_ids,node_ids\n");
      for (TripRecord record : records) {
        StringBuilder edgeStr = new StringBuilder();
        for (int i = 0; i < record.edgeIds.size(); i++) {
          if (i > 0) edgeStr.append(";");
          edgeStr.append(record.edgeIds.get(i));
        }
        StringBuilder nodeStr = new StringBuilder();
        for (int i = 0; i < record.nodeIds.size(); i++) {
          if (i > 0) nodeStr.append(";");
          nodeStr.append(record.nodeIds.get(i));
        }
        writer.write(String.format("%d,%.2f,%.2f,%d,%d,%s,%s\n",
            record.agentId,
            record.startStep,
            record.endStep,
            record.originNodeId,
            record.destNodeId,
            edgeStr.toString(),
            nodeStr.toString()
        ));
      }
      logger.info("[TripRouteRecorder] Successfully saved test_trips file.");
    } catch (IOException e) {
      logger.severe("[TripRouteRecorder] Failed to save test_trips file: " + e.getMessage());
    }
  }
}
