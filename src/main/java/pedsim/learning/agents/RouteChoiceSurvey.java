package pedsim.learning.agents;

import ec.util.MersenneTwisterFast;
import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStream;
import java.io.InputStreamReader;
import java.io.UncheckedIOException;
import java.nio.charset.StandardCharsets;
import pedsim.core.agents.RouteChoiceActivation;
import pedsim.core.agents.RouteChoiceModel.BarrierPreferences;
import pedsim.core.utilities.StringEnum.AgentBarrierType;

/**
 * The route-choice survey's population row, as the source of each learner's own activation
 * probabilities.
 *
 * <p>The survey is Muenster's ({@code Muenster/Muenster_clusters.csv}, row {@code POPULATION}): the
 * share of respondents using each mechanism, with its standard deviation across the clusters. A
 * learner in any city is given a draw of its own from those, once, so it has a route-choice style
 * of its own rather than a fresh coin flip per trip. Taking one city's survey as the population's
 * is an assumption.
 */
public final class RouteChoiceSurvey {

  private static final String RESOURCE = "Muenster/Muenster_clusters.csv";
  private static final String ROW = "POPULATION";

  /** Column order of the clusters file: each quantity is a mean followed by a standard deviation. */
  private static final int USING_ELEMENTS = 1;

  private static final int ONLY_DISTANCE = 5;
  private static final int ONLY_ANGULAR = 7;
  private static final int LOCAL_DISTANCE = 9;
  private static final int LOCAL_ANGULAR = 11;
  private static final int REGIONS = 13;
  private static final int ROUTE_MARKS = 15;
  private static final int BARRIERS = 17;
  private static final int DISTANT_LANDMARKS = 19;
  private static final int PREFERENCE_NATURAL = 21;
  private static final int AVERSION_SEVERING = 23;

  /** Cost multiplier of an edge along a natural barrier at preference 0 and at 1 (see empirical). */
  private static final double NATURAL_AT_NO_PREFERENCE = 1.0;

  private static final double NATURAL_AT_FULL_PREFERENCE = 0.0;

  /** Cost multiplier of an edge along a severing barrier at aversion 0 and at 1. */
  private static final double SEVERING_AT_NO_AVERSION = 1.0;

  private static final double SEVERING_AT_FULL_AVERSION = 2.0;

  private static final double NATURAL_PREFERRED_BELOW = 0.95;
  private static final double SEVERING_AVOIDED_ABOVE = 1.05;

  private static volatile double[] population;

  private RouteChoiceSurvey() {}

  /**
   * Draws one learner's activation: each probability is a normal draw around the survey mean with
   * the survey's standard deviation, cut to [0, 1].
   *
   * @param random the agent's own generator
   * @return the learner's route-choice activation
   */
  public static RouteChoiceActivation draw(MersenneTwisterFast random) {
    double[] row = row();
    double natural =
        rescale(
            sample(row, PREFERENCE_NATURAL, random),
            NATURAL_AT_NO_PREFERENCE,
            NATURAL_AT_FULL_PREFERENCE);
    double severing =
        rescale(
            sample(row, AVERSION_SEVERING, random),
            SEVERING_AT_NO_AVERSION,
            SEVERING_AT_FULL_AVERSION);
    BarrierPreferences barriers =
        new BarrierPreferences(
            AgentBarrierType.SEPARATING,
            natural < NATURAL_PREFERRED_BELOW,
            severing > SEVERING_AVOIDED_ABOVE,
            natural,
            row[PREFERENCE_NATURAL + 1],
            severing,
            row[AVERSION_SEVERING + 1]);
    return new RouteChoiceActivation(
        sample(row, USING_ELEMENTS, random),
        sample(row, ONLY_DISTANCE, random),
        sample(row, ONLY_ANGULAR, random),
        sample(row, LOCAL_DISTANCE, random),
        sample(row, LOCAL_ANGULAR, random),
        sample(row, REGIONS, random),
        sample(row, ROUTE_MARKS, random),
        sample(row, BARRIERS, random),
        sample(row, DISTANT_LANDMARKS, random),
        barriers);
  }

  private static double sample(double[] row, int mean, MersenneTwisterFast random) {
    double sd = row[mean + 1];
    double value = sd <= 0.0 ? row[mean] : row[mean] + random.nextGaussian() * sd;
    return Math.max(0.0, Math.min(1.0, value));
  }

  private static double rescale(double value, double atZero, double atOne) {
    return atZero + value * (atOne - atZero);
  }

  private static double[] row() {
    double[] row = population;
    if (row == null) {
      synchronized (RouteChoiceSurvey.class) {
        if (population == null) {
          population = read();
        }
        row = population;
      }
    }
    return row;
  }

  private static double[] read() {
    try (InputStream in = RouteChoiceSurvey.class.getClassLoader().getResourceAsStream(RESOURCE)) {
      if (in == null) {
        throw new IllegalStateException("route-choice survey not found: " + RESOURCE);
      }
      BufferedReader reader = new BufferedReader(new InputStreamReader(in, StandardCharsets.UTF_8));
      String line;
      while ((line = reader.readLine()) != null) {
        String[] cells = line.split(",", -1);
        if (cells.length >= 25 && cells[0].trim().equals(ROW)) {
          double[] values = new double[cells.length];
          for (int i = 1; i < cells.length; i++) {
            values[i] = cells[i].isBlank() ? 0.0 : Double.parseDouble(cells[i].trim());
          }
          return values;
        }
      }
      throw new IllegalStateException("no " + ROW + " row in " + RESOURCE);
    } catch (IOException e) {
      throw new UncheckedIOException(e);
    }
  }
}
