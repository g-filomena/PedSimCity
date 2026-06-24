package pedsim.core.engine;

import java.util.ArrayList;
import java.util.List;
import java.util.logging.Logger;
import org.locationtech.jts.geom.Coordinate;
import pedsim.core.agents.Agent;
import pedsim.core.parameters.TimePars;
import pedsim.core.utilities.LoggerUtil;
import pedsim.core.website.HtmlExporter;

/**
 * Records a compact snapshot of all walking agents at configurable step intervals.
 *
 * <p>Each snapshot entry is a {@code double[]} of the form:
 * <pre>{step, agentID, lon, lat, vulnerable(1/0)}</pre>
 *
 * <p>The resulting list is consumed by {@link HtmlExporter} at the end of a
 * simulation run to embed trajectory data directly into the self-contained
 * HTML dashboard file.
 */
public class TrajectoryRecorder {

  private static final Logger logger = LoggerUtil.getLogger();

  /**
   * How often (in simulation steps) a snapshot is taken.
   * Increase this value to reduce HTML file size for long runs.
   * Default: every step. Adjust via {@link #setSnapshotInterval(int)}.
   */
  private int snapshotInterval = 1;

  /** The simulation state, used to read the walking agent set. */
  private final PedSimCity state;

  /**
   * Thread-safe list of all trajectory snapshots collected so far.
   * Each entry: [step, agentID, lon, lat, vulnerable(0/1)]
   */
  private final List<double[]> snapshots = new ArrayList<>();

  /**
   * Constructs a new TrajectoryRecorder for the given simulation state.
   *
   * @param state The PedSimCity simulation state to observe.
   */
  public TrajectoryRecorder(PedSimCity state) {
    this.state = state;
  }

  /**
   * Should be called inside the Engine's main step loop.
   * Records all currently walking agents if {@code currentStep} is a
   * multiple of {@link #snapshotInterval}.
   *
   * @param currentStep The current simulation step count.
   */
  public void record(long currentStep) {
    if (currentStep % snapshotInterval != 0) {
      return;
    }

    // Take a safe copy of the walking set to avoid concurrent-modification issues
    List<Agent> walking = new ArrayList<>(state.agentsWalking);

    for (Agent agent : walking) {
      if (agent.getLocation() == null) {
        continue;
      }
      Coordinate coord = agent.getLocation().geometry.getCoordinate();
      if (coord == null) {
        continue;
      }

      snapshots.add(
          new double[] {
            currentStep,
            agent.agentID,
            coord.x, // longitude (or projected X)
            coord.y, // latitude  (or projected Y)
            agent.isVulnerableBoolean() ? 1.0 : 0.0
          });
    }
  }

  /**
   * Returns an unmodifiable view of the collected trajectory snapshots.
   * Each entry is {@code [step, agentID, lon, lat, vulnerable]}.
   *
   * @return The list of trajectory snapshot arrays.
   */
  public List<double[]> getSnapshots() {
    return List.copyOf(snapshots);
  }

  /**
   * Returns the step-to-time string mapping used by the dashboard's slider labels.
   * This converts integer steps back to readable HH:MM format.
   *
   * @param step The simulation step to convert.
   * @return A human-readable time string such as "08:20".
   */
  public static String stepToTimeLabel(long step) {
    return TimePars.getTime(step).toLocalTime().toString().substring(0, 5);
  }

  /**
   * Clears all collected snapshots. Call this before reusing the recorder
   * for a new simulation day or job.
   */
  public void clear() {
    snapshots.clear();
  }

  /**
   * Sets the recording interval.
   *
   * @param interval Number of steps between each snapshot. Must be >= 1.
   */
  public void setSnapshotInterval(int interval) {
    if (interval < 1) {
      throw new IllegalArgumentException("Snapshot interval must be >= 1, got: " + interval);
    }
    this.snapshotInterval = interval;
    logger.info("[TrajectoryRecorder] Snapshot interval set to every " + interval + " step(s).");
  }
}
