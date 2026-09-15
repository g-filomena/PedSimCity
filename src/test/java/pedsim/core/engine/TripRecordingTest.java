package pedsim.core.engine;

import static org.junit.jupiter.api.Assertions.*;

import java.nio.file.Files;
import java.nio.file.Path;
import java.util.List;
import java.util.Locale;
import java.util.concurrent.CompletableFuture;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.io.TempDir;
import org.locationtech.jts.geom.Coordinate;
import org.locationtech.jts.geom.GeometryFactory;
import org.locationtech.jts.planargraph.DirectedEdge;
import pedsim.core.agents.Agent;
import sim.graph.EdgeGraph;
import sim.graph.NodeGraph;

class TripRecordingTest {
  @TempDir Path temp;

  @Test
  void concurrentJobsKeepTripsAndExportsSeparate() throws Exception {
    PedSimCity first = new PedSimCity(1, 0, new ScenarioConfig(null, null));
    PedSimCity second = new PedSimCity(2, 1, new ScenarioConfig(null, null));
    CompletableFuture.allOf(
            CompletableFuture.runAsync(() -> record(first.tripRecorder, 1)),
            CompletableFuture.runAsync(() -> record(second.tripRecorder, 2)))
        .join();
    assertEquals(1, first.tripRecorder.getRecords().size());
    assertEquals(1, second.tripRecorder.getRecords().size());
    assertEquals(1, first.tripRecorder.getRecords().getFirst().agentId);
    assertEquals(2, second.tripRecorder.getRecords().getFirst().agentId);
    Path a = temp.resolve(TripDiagnostic.jobFilename("test_trips.csv", 0));
    Path b = temp.resolve(TripDiagnostic.jobFilename("test_trips.csv", 1));
    first.tripRecorder.saveToFile(a.toString());
    second.tripRecorder.saveToFile(b.toString());
    assertNotEquals(a, b);
    assertTrue(Files.readAllLines(a).get(1).startsWith("1,"));
    assertTrue(Files.readAllLines(b).get(1).startsWith("2,"));
    second.tripRecorder.clear();
    assertEquals(1, first.tripRecorder.getRecords().size());
  }

  @Test
  void abComparisonIncludesPairsBeyondSeventyTwo() throws Exception {
    var recorder = new TripRouteRecorder();
    record(recorder, 198);
    record(recorder, 199);
    Path csv = temp.resolve("ab.csv");
    TripDiagnostic.saveABTestComparison(csv.toString(), recorder.getRecords());
    List<String> lines = Files.readAllLines(csv);
    assertEquals(2, lines.size());
    assertTrue(lines.get(1).startsWith("99,0,"));
    assertFalse(lines.get(1).contains("Day-"));
  }

  @Test
  void csvColumnsRemainStableWithCommaDecimalLocale() throws Exception {
    Locale previous = Locale.getDefault();
    try {
      Locale.setDefault(Locale.GERMANY);
      var recorder = new TripRouteRecorder();
      record(recorder, 1);
      Path raw = temp.resolve("raw.csv");
      Path diagnostic = temp.resolve("diagnostic.csv");
      recorder.saveToFile(raw.toString());
      TripDiagnostic.save(diagnostic.toString(), recorder.getRecords());
      assertEquals(8, Files.readAllLines(raw).get(1).split(",", -1).length);
      assertEquals(9, Files.readAllLines(diagnostic).get(1).split(",", -1).length);
    } finally {
      Locale.setDefault(previous);
    }
  }

  private static void record(TripRouteRecorder recorder, int id) {
    var a = new NodeGraph(new Coordinate(0, 0));
    a.setID(10);
    var b = new NodeGraph(new Coordinate(3, 4));
    b.setID(20);
    var edge =
        new EdgeGraph(
            new GeometryFactory()
                .createLineString(new Coordinate[] {a.getCoordinate(), b.getCoordinate()}));
    edge.setID(30);
    var forward = new DirectedEdge(a, b, b.getCoordinate(), true);
    var reverse = new DirectedEdge(b, a, a.getCoordinate(), false);
    edge.setDirectedEdges(forward, reverse);
    Agent agent = new Agent();
    agent.agentID = id;
    recorder.recordTrip(agent, 1, 2, List.of(forward), id % 2 == 0);
  }
}
