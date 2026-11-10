package pedsim.learning.cognitivemap;

/**
 * Running totals of the thresholds an agent's map has handed out and the vividness each followed,
 * so a run can report what recognition actually saw.
 */
public class RecognitionStats {

  private long queries;
  private double vividnessSum;
  private double thresholdSum;

  void record(double vividness, double threshold) {
    queries++;
    vividnessSum += vividness;
    thresholdSum += threshold;
  }

  /** The totals so far, which are then cleared. */
  RecognitionStats drain() {
    RecognitionStats taken = new RecognitionStats();
    taken.queries = queries;
    taken.vividnessSum = vividnessSum;
    taken.thresholdSum = thresholdSum;
    queries = 0;
    vividnessSum = 0.0;
    thresholdSum = 0.0;
    return taken;
  }

  /** Adds another set of totals to these. */
  public void add(RecognitionStats other) {
    queries += other.queries;
    vividnessSum += other.vividnessSum;
    thresholdSum += other.thresholdSum;
  }

  public long queries() {
    return queries;
  }

  /** The mean vividness queried, or NaN when nothing was. */
  public double meanVividness() {
    return queries == 0 ? Double.NaN : vividnessSum / queries;
  }

  /** The mean threshold handed out, or NaN when nothing was. */
  public double meanThreshold() {
    return queries == 0 ? Double.NaN : thresholdSum / queries;
  }
}
