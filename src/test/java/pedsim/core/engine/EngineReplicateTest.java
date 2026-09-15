package pedsim.core.engine;

import static org.junit.jupiter.api.Assertions.*;

import java.util.List;
import java.util.concurrent.CopyOnWriteArrayList;
import java.util.logging.Handler;
import java.util.logging.LogRecord;
import org.junit.jupiter.api.Test;
import pedsim.core.parameters.Pars;
import pedsim.core.parameters.TimePars;
import pedsim.core.utilities.LoggerUtil;
import pedsim.testing.ParameterSnapshot;

class EngineReplicateTest {
  @Test
  void parallelWorkersContributeToTheParentSummary() throws Exception {
    List<String> messages = new CopyOnWriteArrayList<>();
    Handler handler =
        new Handler() {
          public void publish(LogRecord record) {
            messages.add(record.getMessage());
          }

          public void flush() {}

          public void close() {}
        };
    LoggerUtil.getLogger().addHandler(handler);
    try (var saved = new ParameterSnapshot(Pars.class, TimePars.class)) {
      Pars.jobs = 2;
      new ProbeEngine().runJobs(new ScenarioConfig(null, null), true);
      String summary =
          messages.stream().filter(s -> s.startsWith("Replicates:")).findFirst().orElseThrow();
      assertTrue(summary.contains("job 0 (seed 100)"));
      assertTrue(summary.contains("job 1 (seed 101)"));
      assertTrue(summary.contains("over 2 replicates"));
      assertFalse(SimulationStateStore.getInstance().running);
    } finally {
      LoggerUtil.getLogger().removeHandler(handler);
    }
  }

  @Test
  void failedParallelJobDoesNotReleaseReservationWhileAnotherWorkerIsRunning() throws Exception {
    var waiting = new java.util.concurrent.CountDownLatch(1);
    var release = new java.util.concurrent.CountDownLatch(1);
    class FailingEngine extends ProbeEngine {
      protected Engine createWorkerEngine() {
        return new FailingEngine();
      }

      public void executeJob(int job, ScenarioConfig config) {
        if (job == 0) throw new IllegalStateException("job failed");
        waiting.countDown();
        try {
          if (!release.await(5, java.util.concurrent.TimeUnit.SECONDS))
            throw new AssertionError("worker timeout");
        } catch (InterruptedException e) {
          throw new RuntimeException(e);
        }
      }
    }
    try (var saved = new ParameterSnapshot(Pars.class, TimePars.class)) {
      Pars.jobs = 2;
      var result =
          java.util.concurrent.CompletableFuture.runAsync(
              () -> {
                try {
                  new FailingEngine().runJobs(new ScenarioConfig(null, null), true);
                } catch (Exception e) {
                  throw new java.util.concurrent.CompletionException(e);
                }
              });
      try {
        assertTrue(waiting.await(5, java.util.concurrent.TimeUnit.SECONDS));
        assertFalse(result.isDone());
        assertNull(SimulationStateStore.getInstance().tryReserveRun());
      } finally {
        release.countDown();
        assertThrows(
            java.util.concurrent.ExecutionException.class,
            () -> result.get(5, java.util.concurrent.TimeUnit.SECONDS));
      }
      try (var next = SimulationStateStore.getInstance().tryReserveRun()) {
        assertNotNull(next);
      }
    }
  }

  private static class ProbeEngine extends Engine {
    ProbeEngine() {
      super(PedSimCity::new, 100);
    }

    protected Import createImporter() {
      return new Import() {
        public void importFiles() {}
      };
    }

    protected void clearStaticData() {}

    protected void prepareEnvironment() {}

    protected Engine createWorkerEngine() {
      return new ProbeEngine();
    }

    public void executeJob(int job, ScenarioConfig config) {
      PedSimCity state = stateFactory.create(seedForJob(job), job, config);
      recordJobTotals(job, seedForJob(job), state);
    }
  }
}
