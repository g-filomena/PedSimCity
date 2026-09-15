package pedsim.core.website;

import static org.junit.jupiter.api.Assertions.*;

import java.net.URI;
import java.net.http.HttpClient;
import java.net.http.HttpRequest;
import java.net.http.HttpResponse;
import java.nio.file.Files;
import java.nio.file.Path;
import java.time.Duration;
import java.util.Map;
import java.util.concurrent.CountDownLatch;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicReference;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.io.TempDir;
import pedsim.core.engine.*;
import pedsim.core.parameters.ParameterManager;
import pedsim.core.parameters.Pars;
import pedsim.testing.ParameterSnapshot;

class SimulationRestApiTest {
  @TempDir Path temp;

  @Test
  void dashboardAssetsCannotEscapeTheirRoot() throws Exception {
    Path root = Files.createDirectory(temp.resolve("public"));
    Path dashboard = Files.writeString(root.resolve("dashboard.html"), "dashboard");
    Path image = Files.writeString(root.resolve("map.png"), "image");
    Files.writeString(temp.resolve("outside.png"), "private");
    assertEquals(dashboard.toRealPath(), SimulationRestApi.resolveDashboardFile(root, "/"));
    assertEquals(image.toRealPath(), SimulationRestApi.resolveDashboardFile(root, "/map.png"));
    assertNull(SimulationRestApi.resolveDashboardFile(root, "/../outside.png"));
    assertNull(SimulationRestApi.resolveDashboardFile(root, "/..\\outside.png"));
    assertNull(SimulationRestApi.resolveDashboardFile(root, "//outside.png"));
    assertNull(SimulationRestApi.resolveDashboardFile(root, "/C:/outside.png"));
    assertNull(SimulationRestApi.resolveDashboardFile(root, "/missing.png"));
  }

  @Test
  void secondPostIsRejectedBeforeItCanChangeRunParameters() throws Exception {
    CountDownLatch configuring = new CountDownLatch(1);
    CountDownLatch proceed = new CountDownLatch(1);
    AtomicReference<Thread> worker = new AtomicReference<>();
    AtomicReference<String> loadedCity = new AtomicReference<>();
    AtomicReference<Long> appliedSeed = new AtomicReference<>();
    SimulationModule module =
        new SimulationModule() {
          public String moduleId() {
            return "probe";
          }

          public void clearStaticData() {}

          public void applyDefaults(Map<String, String> params) {
            worker.set(Thread.currentThread());
            configuring.countDown();
            try {
              if (!proceed.await(10, TimeUnit.SECONDS)) throw new AssertionError("setup timeout");
            } catch (InterruptedException e) {
              throw new RuntimeException(e);
            }
          }

          public void loadCityConfig(String city) {
            loadedCity.set(city);
          }

          public void applyParameters(Map<String, Object> params) {}

          public ScenarioConfig scenarioConfig() {
            return new ScenarioConfig(null, null);
          }

          public Engine createEngine() {
            appliedSeed.set(Pars.seed);
            return new Engine(PedSimCity::new) {
              public void runJobs(
                  ScenarioConfig config,
                  boolean parallel,
                  SimulationStateStore.RunReservation reservation) {
                reservation.requireActive();
              }
            };
          }
        };
    try (var saved = new ParameterSnapshot(Pars.class, ParameterManager.class);
        var client = HttpClient.newHttpClient()) {
      Pars.cityName = "BeforeSetup";
      SimulationRestApi.registerModule(module);
      SimulationRestApi.start(0);
      URI uri = URI.create("http://localhost:" + SimulationRestApi.boundPort() + "/api/start");
      try {
        assertEquals(200, post(client, uri, "{\"cityName\":\"First\",\"seed\":123}"));
        assertTrue(configuring.await(5, TimeUnit.SECONDS));
        assertEquals(409, post(client, uri, "{\"cityName\":\"Second\",\"seed\":456}"));
        assertEquals("BeforeSetup", Pars.cityName);
        assertTrue(SimulationStateStore.getInstance().running);
      } finally {
        proceed.countDown();
        Thread thread = worker.get();
        if (thread != null) {
          thread.join(5000);
          assertFalse(thread.isAlive());
        }
        SimulationRestApi.stop();
      }
      assertEquals("First", loadedCity.get());
      assertEquals(123L, appliedSeed.get());
      assertFalse(SimulationStateStore.getInstance().running);
    }
  }

  private static int post(HttpClient client, URI uri, String json) throws Exception {
    return client
        .send(
            HttpRequest.newBuilder(uri)
                .timeout(Duration.ofSeconds(5))
                .header("Content-Type", "application/json")
                .POST(HttpRequest.BodyPublishers.ofString(json))
                .build(),
            HttpResponse.BodyHandlers.discarding())
        .statusCode();
  }
}
