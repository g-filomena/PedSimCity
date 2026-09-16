package pedsim.core.engine;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.nio.file.Files;
import java.nio.file.Path;
import java.util.List;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.TestInstance;
import org.junit.jupiter.api.io.TempDir;
import pedsim.core.launcher.ModuleLauncher;
import pedsim.core.parameters.Pars;

/**
 * The same seed twice gives the same run, and a different seed does not.
 *
 * <p>This is the test that catches a {@code Gateway}-style tie-break the day it appears rather than
 * the week after. Every non-reproducibility this repository has had was of one shape — a collection
 * keyed on an object with no {@code hashCode}, iterated somewhere that a draw, a first-match scan or
 * a stable sort turned into a decision — and none of them announced itself. They were found by
 * running the same seed twice and diffing, which is exactly what this does, so the method is in the
 * suite instead of in somebody's memory.
 *
 * <p><b>Why there are two assertions.</b> "Two runs agree" passes trivially if the trace records
 * nothing that any decision could move — an empty file agrees with an empty file, and a file that
 * only lists agent IDs agrees whatever the routing did. The second test runs a <i>different</i> seed
 * and requires the trace to change, so the first one cannot be vacuous. A test that has never been
 * seen to fail is a test nobody has tested, and this is that check built in rather than done once by
 * hand.
 *
 * <p><b>Same JVM, not two machines.</b> A second run in this JVM re-derives every identity hash the
 * same way, so this cannot see the cross-machine class of defect that {@code PoiClassifier}'s
 * attraction maps were (closed 14 September 2026 — see {@code CLAUDE.md}). It sees the larger
 * family: anything that depends on wall-clock time, on an unseeded generator, on thread interleaving,
 * or on state left behind by the previous run. The cross-machine check is this same trace file
 * compared against one produced elsewhere, which needs two machines and so cannot live here.
 *
 * <p><b>Why the trace and not the totals.</b> Two runs can walk the same total metres by different
 * routes. {@code RouteTrace}'s per-leg file is written at {@code Agent.initialiseRoute()}, the one
 * seam every planner crosses, carries no timestamps, and names the scenario, agent, trip, origin,
 * destination, node and edge counts and length of each leg — so a diff answers "did anything change"
 * directly, and the first differing line names the decision that moved.
 *
 * <p>Tagged {@code slow}: it runs three simulated days end to end. {@code mvn test -Pslow-tests}.
 */
@Tag("slow")
@TestInstance(TestInstance.Lifecycle.PER_CLASS)
class SeedReproducesRunTest {

  /**
   * Small enough that three full runs finish inside a test, real enough to route: {@code Muenster}
   * is the smallest bundled city addressable by {@code --cityName}.
   */
  private static final String CITY = "Muenster";

  private static final String SEED = "20260912";

  /** Any other seed. Its only job is to prove the trace responds to the seed at all. */
  private static final String OTHER_SEED = "20260913";

  // Static: an instance @TempDir is created per test method, after @BeforeAll has run.
  @TempDir static Path shared;

  private List<String> first;
  private List<String> second;
  private List<String> otherSeed;

  @BeforeAll
  void runTheSameDayThreeTimes() throws Exception {
    first = runCore(shared.resolve("first.tsv"), SEED);
    second = runCore(shared.resolve("second.tsv"), SEED);
    otherSeed = runCore(shared.resolve("other.tsv"), OTHER_SEED);

    assertTrue(
        first.size() > 1,
        "the run planned no legs, so every comparison in this class would be between empty files");
  }

  @Test
  void theSameSeedTwiceWritesTheSameTrace() {
    int differingLine = firstDifference(first, second);
    assertEquals(
        -1,
        differingLine,
        () ->
            "two runs of seed "
                + SEED
                + " diverged at trace line "
                + (differingLine + 1)
                + ", which names the decision that moved:\n  run 1: "
                + first.get(differingLine)
                + "\n  run 2: "
                + second.get(differingLine)
                + "\nThe usual cause is a collection keyed on a graph object or on an Agent whose "
                + "iteration order reaches a draw, a first-match scan or a stable sort. See the "
                + "invariant in core/TODO.md.");
    assertEquals(
        first.size(),
        second.size(),
        "the two runs planned a different number of legs on the same seed");
  }

  /**
   * The control that stops the check above from passing on a trace nothing can move.
   *
   * <p>If this fails, the per-leg record has stopped recording something a decision depends on —
   * which is how {@code RouteTrace} could once report "planned 0 m, walked 0 m" for every run while
   * looking perfectly healthy.
   */
  @Test
  void aDifferentSeedWritesADifferentTrace() {
    assertNotEquals(
        -1,
        firstDifference(first, otherSeed),
        "seeds "
            + SEED
            + " and "
            + OTHER_SEED
            + " produced identical traces, so the trace records nothing the seed can change and "
            + "the reproducibility check above proves nothing");
  }

  /** Index of the first line on which the two traces disagree, or -1 when they do not. */
  private static int firstDifference(List<String> a, List<String> b) {
    for (int line = 0; line < Math.min(a.size(), b.size()); line++) {
      if (!a.get(line).equals(b.get(line))) {
        return line;
      }
    }
    return a.size() == b.size() ? -1 : Math.min(a.size(), b.size());
  }

  /**
   * One headless core day, with the per-leg trace pointed at {@code traceFile}.
   *
   * <p>Goes through {@link ModuleLauncher}, the entry point a real run uses, so the test cannot pass
   * on a configuration path no run takes. The trace destination is a system property because that is
   * how {@code RouteTrace} is switched on; it is cleared afterwards so a later test in the same JVM
   * does not inherit it.
   */
  private static List<String> runCore(Path traceFile, String seed) throws Exception {
    String previous = System.getProperty("pedsim.trace");
    System.setProperty("pedsim.trace", traceFile.toString());
    try {
      ModuleLauncher.run(
          CoreSimulationModule.INSTANCE,
          new String[] {
            "--headless",
            "--cityName=" + CITY,
            "--seed=" + seed,
            "--numAgents=60",
            "--days=1",
            "--jobs=1",
            "--stepDelayMs=0",
            "--exportHtmlDashboard=false"
          });
    } finally {
      if (previous == null) {
        System.clearProperty("pedsim.trace");
      } else {
        System.setProperty("pedsim.trace", previous);
      }
      Pars.cityName = CITY;
    }
    return Files.readAllLines(traceFile);
  }
}
