package pedsim.activity.agents;

import ec.util.MersenneTwisterFast;
import java.time.DayOfWeek;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import pedsim.activity.parameters.ActivityPars;

/**
 * A coarse socio-demographic persona: it determines whether and when the agent attends a mandatory
 * activity (work / study), its walking speed, and the mix of discretionary activities it prefers.
 *
 * <p>Shares are configured in {@link ActivityPars}; assignment happens in {@code ActivityPopulate}.
 */
public enum Persona {

  /** Full-time worker: commutes on weekdays, morning start window. */
  WORKER(1.05, 6.5, 10.5, 360, 540, new double[] {0.25, 0.15, 0.20, 0.10, 0.25, 0.05}),

  /** Student: attends the study place on weekdays, tighter start window, shorter stay. */
  STUDENT(1.10, 7.5, 9.5, 300, 420, new double[] {0.15, 0.05, 0.20, 0.20, 0.30, 0.10}),

  /** Retiree: no mandatory activity, slower walking, daytime errands/leisure oriented. */
  RETIREE(0.75, Double.NaN, Double.NaN, 0, 0, new double[] {0.25, 0.30, 0.05, 0.00, 0.20, 0.20}),

  /** Flexible / non-employed adult: no mandatory activity, balanced mix. */
  FLEX(1.00, Double.NaN, Double.NaN, 0, 0, new double[] {0.25, 0.15, 0.15, 0.10, 0.25, 0.10});

  /** Discretionary purposes, in the order of each persona's preference-weight array. */
  private static final ActivityPurpose[] DISCRETIONARY = {
    ActivityPurpose.SHOPPING,
    ActivityPurpose.ERRANDS,
    ActivityPurpose.DINING,
    ActivityPurpose.NIGHTLIFE,
    ActivityPurpose.LEISURE,
    ActivityPurpose.STROLL
  };

  private final double speedFactor;
  private final double mandatoryStartEarliest; // hour of day; NaN = no mandatory activity
  private final double mandatoryStartLatest;
  private final int mandatoryStayMinMinutes;
  private final int mandatoryStayMaxMinutes;
  private final double[] purposeWeights;

  Persona(
      double speedFactor,
      double mandatoryStartEarliest,
      double mandatoryStartLatest,
      int mandatoryStayMinMinutes,
      int mandatoryStayMaxMinutes,
      double[] purposeWeights) {
    this.speedFactor = speedFactor;
    this.mandatoryStartEarliest = mandatoryStartEarliest;
    this.mandatoryStartLatest = mandatoryStartLatest;
    this.mandatoryStayMinMinutes = mandatoryStayMinMinutes;
    this.mandatoryStayMaxMinutes = mandatoryStayMaxMinutes;
    this.purposeWeights = purposeWeights;
  }

  /** Earliest hour this persona's mandatory activity can start; NaN when it has none. */
  public double getMandatoryStartEarliest() {
    return mandatoryStartEarliest;
  }

  /** Latest hour this persona's mandatory activity can start; NaN when it has none. */
  public double getMandatoryStartLatest() {
    return mandatoryStartLatest;
  }

  /** This persona's preference weights over the discretionary purposes, in their fixed order. */
  public double[] getPurposeWeights() {
    return purposeWeights.clone();
  }

  /**
   * The purposes this persona weights most heavily, strongest first.
   *
   * <p>Used to anchor the cognitive map of someone with no workplace on the places they would
   * actually go, rather than leaving them with a known world one neighbourhood wide.
   *
   * @param count how many to return
   * @return up to {@code count} purposes, strongest first
   */
  public List<ActivityPurpose> strongestPurposes(int count) {
    Integer[] order = new Integer[DISCRETIONARY.length];
    for (int i = 0; i < order.length; i++) {
      order[i] = i;
    }
    Arrays.sort(order, (a, b) -> Double.compare(purposeWeights[b], purposeWeights[a]));
    List<ActivityPurpose> strongest = new ArrayList<>(count);
    for (int i = 0; i < Math.min(count, order.length); i++) {
      if (purposeWeights[order[i]] > 0.0) {
        strongest.add(DISCRETIONARY[order[i]]);
      }
    }
    return strongest;
  }

  /** Base walking-speed multiplier for this persona (individual noise is added per agent). */
  public double getSpeedFactor() {
    return speedFactor;
  }

  /** Whether this persona has a mandatory daily activity (work or study) at all. */
  public boolean hasMandatoryActivity() {
    return !Double.isNaN(mandatoryStartEarliest);
  }

  /** Whether this persona attends its mandatory activity on the given day (weekdays only). */
  public boolean worksOn(DayOfWeek day) {
    return hasMandatoryActivity()
        && day != DayOfWeek.SATURDAY
        && day != DayOfWeek.SUNDAY;
  }

  /**
   * Whether the given hour falls inside this persona's mandatory-activity start window. An agent
   * released after the window never starts work that day (no more 5 PM work departures).
   */
  public boolean isWithinMandatoryStartWindow(double hourOfDay) {
    return hasMandatoryActivity()
        && hourOfDay >= mandatoryStartEarliest
        && hourOfDay <= mandatoryStartLatest;
  }

  /** Stay duration (minutes) at the mandatory activity: uniform within the persona's range. */
  public int sampleMandatoryStayMinutes(MersenneTwisterFast random) {
    if (!hasMandatoryActivity()) {
      return 0;
    }
    return mandatoryStayMinMinutes
        + random.nextInt(mandatoryStayMaxMinutes - mandatoryStayMinMinutes + 1);
  }

  /**
   * Samples a discretionary purpose from this persona's preference mix, restricted to purposes
   * whose opening window contains the given hour. Falls back to {@link ActivityPurpose#STROLL}
   * (always open) when nothing else is available.
   */
  public ActivityPurpose sampleDiscretionaryPurpose(double hourOfDay, MersenneTwisterFast random) {
    double total = 0.0;
    for (int i = 0; i < DISCRETIONARY.length; i++) {
      if (DISCRETIONARY[i].isOpenAt(hourOfDay)) {
        total += purposeWeights[i];
      }
    }
    if (total <= 0.0) {
      return ActivityPurpose.STROLL;
    }
    double r = random.nextDouble() * total;
    double cumulative = 0.0;
    for (int i = 0; i < DISCRETIONARY.length; i++) {
      if (!DISCRETIONARY[i].isOpenAt(hourOfDay)) {
        continue;
      }
      cumulative += purposeWeights[i];
      if (r <= cumulative) {
        return DISCRETIONARY[i];
      }
    }
    return ActivityPurpose.STROLL;
  }

  /**
   * Release affinity by hour of day: the acceptance probability, in {@code (0, 1]}, that an
   * at-home agent of this persona is picked by the release manager at that hour. Commuter
   * personas dominate the morning/evening peaks, retirees the middle of the day; the agenda
   * system independently shapes *what* they do once out.
   */
  public double releaseAffinity(int hour) {
    return switch (this) {
      case WORKER -> {
        if (hour >= 6 && hour <= 9) yield 1.0; // morning commute
        if (hour >= 16 && hour <= 19) yield 1.0; // evening peak
        if (hour >= 10 && hour <= 15) yield 0.45;
        if (hour >= 20 && hour <= 23) yield 0.6;
        yield 0.3; // small hours
      }
      case STUDENT -> {
        if (hour >= 7 && hour <= 9) yield 1.0; // school run
        if (hour >= 14 && hour <= 17) yield 1.0; // after classes
        if (hour >= 10 && hour <= 13) yield 0.6;
        if (hour >= 18 && hour <= 22) yield 0.7;
        yield 0.3;
      }
      case RETIREE -> {
        if (hour >= 9 && hour <= 17) yield 1.0; // daytime errands/leisure
        if (hour >= 6 && hour <= 8) yield 0.6;
        if (hour >= 18 && hour <= 20) yield 0.5;
        yield 0.2;
      }
      case FLEX -> {
        if (hour >= 9 && hour <= 22) yield 1.0;
        if (hour >= 6 && hour <= 8) yield 0.8;
        yield 0.4;
      }
    };
  }

  /** Samples a persona according to the global shares configured in {@link ActivityPars}. */
  public static Persona sample(MersenneTwisterFast random) {
    return sample(random, Double.NaN, Double.NaN, Double.NaN);
  }

  /**
   * Samples a persona conditioned on the home zone's census structure.
   *
   * <p>Three of the four shares are read off the census: retirees from the 65+ age band, students
   * from the 15-24 band, workers from ISTAT P101, the residents aged 15-64 in employment. Flex is
   * what is left, which is what flex means here - the 15-64 who are neither employed nor of
   * student age. It replaces a global 77/23 worker-to-flex ratio that had no source and was
   * applied identically to every zone, so who has a job was the one persona fact the census was
   * not allowed to settle.
   *
   * <p>One overlap survives, and it is bounded. P101 counts the employed 15-24, who are already
   * counted here as students, so taking it whole borrows them from flex rather than from student.
   * The 2021 permanent census publishes no enrolment variable at section level - only educational
   * attainment - so there is no per-zone figure for the employed 15-24 to subtract. The error is
   * at most the youth employment rate times the 15-24 share, a couple of points of the adult
   * population, and it moves people between two personas that both lack a commute.
   *
   * <p>{@code NaN} shares fall back to the global {@link ActivityPars} values one by one, so a
   * census carrying age bands but no employment keeps the old worker/flex ratio and nothing else
   * changes.
   */
  public static Persona sample(
      MersenneTwisterFast random,
      double zoneRetireeShare,
      double zoneStudentShare,
      double zoneWorkerShare) {
    double retiree =
        Double.isNaN(zoneRetireeShare)
            ? ActivityPars.retireeShare
            : Math.min(1.0, Math.max(0.0, zoneRetireeShare));
    double student =
        Double.isNaN(zoneStudentShare)
            ? ActivityPars.studentShare
            : Math.min(1.0, Math.max(0.0, zoneStudentShare));

    double residual = Math.max(0.0, 1.0 - retiree - student);
    double worker;
    if (Double.isNaN(zoneWorkerShare)) {
      double workerFlexTotal = ActivityPars.workerShare + ActivityPars.flexShare;
      worker =
          workerFlexTotal > 0.0
              ? residual * ActivityPars.workerShare / workerFlexTotal
              : residual / 2.0;
    } else {
      // Capped at the residual: the employed of student age are already counted as students, and
      // a zone whose employment share exceeds what is left cannot have more workers than adults.
      worker = Math.min(residual, Math.max(0.0, zoneWorkerShare));
    }
    double flex = residual - worker;

    double total = worker + student + retiree + flex;
    if (total <= 0.0) {
      return WORKER;
    }
    double r = random.nextDouble() * total;
    if ((r -= worker) < 0) {
      return WORKER;
    }
    if ((r -= student) < 0) {
      return STUDENT;
    }
    if ((r -= retiree) < 0) {
      return RETIREE;
    }
    return FLEX;
  }
}
