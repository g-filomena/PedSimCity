package pedsim.activity.engine;

import java.time.LocalDateTime;
import java.util.ArrayList;
import java.util.List;
import pedsim.activity.agents.ActivityAgent;
import pedsim.activity.agents.DailyAgenda;
import pedsim.activity.agents.DepartureProfile;
import pedsim.activity.agents.Persona;
import pedsim.activity.parameters.ActivityPars;
import pedsim.core.agents.Agent;
import pedsim.core.engine.BaselineTravelDemand;
import pedsim.core.parameters.Pars;
import pedsim.core.parameters.TimePars;

/**
 * Travel demand for the activity tier: a count of trip chains, a departure profile built from what
 * people have to do, a persona affinity, weather, and a walk-share mode split.
 *
 * <p>Gathered here rather than on the simulation state so that the questions the release manager
 * asks have one home, and a change to any of them is visibly a change to travel demand.
 */
public class ActivityTravelDemand extends BaselineTravelDemand {

  private final PedSimCityActivity activity;

  /** Everything below is rebuilt when the simulated day changes. */
  private long preparedDay = Long.MIN_VALUE;

  private DepartureProfile departureProfile;

  private double discretionaryChainsPerPerson;

  // The day's arithmetic, kept so the day's output row can state it rather than re-deriving it.
  private double dayMandatoryLegs;
  private double dayBudgetLegs;
  private double dayLegsPerChain;
  private double dayWorkerWalkShare;
  private double dayStudentWalkShare;

  public ActivityTravelDemand(PedSimCityActivity state) {
    super(state);
    this.activity = state;
  }

  /**
   * Settles the day: who has to go somewhere and when, and how much travel is left over for
   * everything else.
   *
   * <p>One pass over the population. Each agent draws its mandatory departure minute, or
   * establishes that it has none today - it has no job, it does not walk to it, or its persona does
   * not attend on this day. The legs today's commutes account for, <i>whatever mode they are made
   * by</i>, are subtracted from the day's trip budget, and what remains is bought as discretionary
   * chains.
   *
   * <p>The budget is {@link ActivityPars#tripsPerPersonPerDay}, an ISFORT count of <i>legs</i> at
   * every mode, divided by the chain length the agendas will actually produce - computed here from
   * the realised persona mix, not from a constant. A chain length belongs to the model and changes
   * when the agenda probabilities change, so it has no business being baked into a number
   * attributed to a travel survey.
   *
   * <p>Nothing here decides how many trips are walked. That is settled trip by trip, once each
   * destination is known, by {@link #walkProbability(double)}; the share it produces is reported by
   * {@code DaySummary} against the survey.
   *
   * <p>Synchronised and idempotent per day; the first release event of a day pays for it.
   */
  private synchronized void prepare(LocalDateTime time) {
    long day = time.toLocalDate().toEpochDay();
    if (day == preparedDay) {
      return;
    }
    preparedDay = day;

    java.time.LocalDate today = time.toLocalDate();
    boolean rainy = activity.isRainyNow();

    double[] personaCounts = new double[4];
    double population = 0.0;
    double mandatoryLegs = 0.0;
    double commuteLegs = 0.0;
    int workersWithJob = 0;
    int workersWalking = 0;
    int studentsWithPlace = 0;
    int studentsWalking = 0;
    int[] walkedCommuteBands = new int[4];

    for (Agent agent : activity.agentsList) {
      if (!(agent instanceof ActivityAgent activityAgent)) {
        continue;
      }
      Persona persona = activityAgent.getPersona();
      if (persona != null && persona.ordinal() < personaCounts.length) {
        personaCounts[persona.ordinal()]++;
        population++;
      }
      if (activityAgent.hasWorkplace()) {
        if (persona == Persona.WORKER) {
          workersWithJob++;
          if (activityAgent.walksToWork()) {
            workersWalking++;
          }
        } else if (persona == Persona.STUDENT) {
          studentsWithPlace++;
          if (activityAgent.walksToWork()) {
            studentsWalking++;
          }
        }
        if (activityAgent.walksToWork()) {
          // Straight line scaled by the measured circuity - the model's own estimate of walked
          // metres, since the route does not exist until the agent sets off. Bands are ISTAT's
          // duration bands at Pars.pedestrianSpeed; see COMMUTE_DISTANCE.md.
          double metres = activityAgent.commuteMetres();
          int band = metres <= 1278.0 ? 0 : metres <= 2556.0 ? 1 : metres <= 5112.0 ? 2 : 3;
          walkedCommuteBands[band]++;
        }
      }
      if (activityAgent.commutesOn(today)) {
        commuteLegs += DailyAgenda.expectedLegs(persona, true, rainy);
      }
      if (activityAgent.planMandatoryDeparture(today)) {
        mandatoryLegs += DailyAgenda.expectedLegs(persona, true, rainy);
      }
    }

    double[] personaShares = new double[personaCounts.length];
    if (population > 0.0) {
      for (int i = 0; i < personaCounts.length; i++) {
        personaShares[i] = personaCounts[i] / population;
      }
    }
    departureProfile = DepartureProfile.discretionary(personaShares);

    // Chain length from the mix the population actually got, weighted by it.
    double legsPerChain = 0.0;
    for (int i = 0; i < personaShares.length; i++) {
      if (personaShares[i] > 0.0) {
        legsPerChain += personaShares[i] * DailyAgenda.expectedLegs(PERSONAS[i], false, rainy);
      }
    }
    legsPerChain = Math.max(1.0, legsPerChain);

    double agents = Pars.numAgents > 0 ? Pars.numAgents : population;
    double budgetLegs = agents * ActivityPars.tripsPerPersonPerDay;
    // Charged at every mode: somebody who drives to work has still made those trips, and the budget
    // counts trips. Subtracting only the walked ones would spend a driver's commute a second time,
    // as somebody else's discretionary walk.
    double discretionaryLegs = Math.max(0.0, budgetLegs - commuteLegs);
    discretionaryChainsPerPerson = agents > 0.0 ? discretionaryLegs / legsPerChain / agents : 0.0;

    dayMandatoryLegs = mandatoryLegs;
    dayBudgetLegs = budgetLegs;
    dayLegsPerChain = legsPerChain;
    dayWorkerWalkShare = workersWithJob > 0 ? (double) workersWalking / workersWithJob : 0.0;
    dayStudentWalkShare =
        studentsWithPlace > 0 ? (double) studentsWalking / studentsWithPlace : 0.0;

    LOGGER.info(
        String.format(
            "day %s: %.0f commute legs (%.0f of them walked) of a %.0f-leg budget; %.3f"
                + " discretionary chains per person at %.2f legs each",
            time.toLocalDate(),
            commuteLegs,
            mandatoryLegs,
            budgetLegs,
            discretionaryChainsPerPerson,
            legsPerChain));

    // The walked commute share is produced by the model, not given to it, so it can be logged
    // against the observed figure. A large gap is a statement about the walk-share curve, or about
    // where the model is putting workplaces — not a number to adjust.
    if (workersWithJob > 0 || studentsWithPlace > 0) {
      LOGGER.info(
          String.format(
              "walked commute share: workers %.1f%% (ISTAT %.1f%%), students %.1f%% (ISTAT %.1f%%)",
              workersWithJob > 0 ? 100.0 * workersWalking / workersWithJob : 0.0,
              100.0 * ActivityPars.walkShareCommuteWorker,
              studentsWithPlace > 0 ? 100.0 * studentsWalking / studentsWithPlace : 0.0,
              100.0 * ActivityPars.walkShareCommuteStudent));

      // The shape, not just the share. Matching 16.3% by pushing workplaces out of reach while the
      // walked commutes come out too long would be worse than the present state, so both are
      // reported. ISTAT (Turin, intra-municipal, work): 76.2 / 19.0 / 3.4 / 1.3.
      int walked =
          walkedCommuteBands[0]
              + walkedCommuteBands[1]
              + walkedCommuteBands[2]
              + walkedCommuteBands[3];
      if (walked > 0) {
        LOGGER.info(
            String.format(
                "walked commute length: <=1278m %.1f%% | 1278-2556m %.1f%% | 2556-5112m %.1f%% |"
                    + " >5112m %.1f%%  (ISTAT work 76.2 / 19.0 / 3.4 / 1.3)",
                100.0 * walkedCommuteBands[0] / walked,
                100.0 * walkedCommuteBands[1] / walked,
                100.0 * walkedCommuteBands[2] / walked,
                100.0 * walkedCommuteBands[3] / walked));
      }
    }
  }

  /** Legs today's structural commutes will walk. */
  public double mandatoryLegs() {
    return dayMandatoryLegs;
  }

  /** The day's whole leg budget, structural and discretionary together. */
  public double budgetLegs() {
    return dayBudgetLegs;
  }

  /** Legs in a discretionary chain, as the realised persona mix produces it. */
  public double legsPerChain() {
    return dayLegsPerChain;
  }

  /** Discretionary chains bought per person today. */
  public double discretionaryChainsPerPerson() {
    return discretionaryChainsPerPerson;
  }

  /** Share of workers with a job who walk to it. */
  public double workerWalkShare() {
    return dayWorkerWalkShare;
  }

  /** Share of students with a place who walk to it. */
  public double studentWalkShare() {
    return dayStudentWalkShare;
  }

  private static final Persona[] PERSONAS = {
    Persona.WORKER, Persona.STUDENT, Persona.RETIREE, Persona.FLEX
  };

  private static final java.util.logging.Logger LOGGER =
      pedsim.core.utilities.LoggerUtil.getLogger();

  /**
   * Departures timed by the agenda system: opening hours and persona preferences, instead of the
   * tuned peaks of {@code TimePars.computeTimeStepShare}. Discretionary only - the commute has its
   * own departure, per agent.
   */
  @Override
  public double departureShare(LocalDateTime time) {
    if (!ActivityPars.useAgendaDepartureProfile) {
      return super.departureShare(time);
    }
    prepare(time);
    return departureProfile.share(time);
  }

  /** Agents whose mandatory departure minute falls inside this release event's window. */
  @Override
  public List<Agent> scheduledDepartures(LocalDateTime time) {
    prepare(time);
    int from = time.getHour() * 60 + time.getMinute();
    int to = from + TimePars.releaseAgentsEveryMinutes;
    List<Agent> due = new ArrayList<>();
    for (Agent agent : activity.agentsAtHome) {
      if (agent instanceof ActivityAgent activityAgent) {
        int minute = activityAgent.mandatoryDepartureMinute();
        if (minute >= from && minute < to) {
          due.add(agent);
        }
      }
    }
    return due;
  }

  /**
   * One departure sets a whole trip chain in motion, so the count handed to the release manager is
   * a count of chains. The conversion from the day's leg budget to chains is this module's - see
   * {@link #prepare} - and core is told only how often somebody sets off.
   */
  @Override
  public double unscheduledDeparturesPerPerson(LocalDateTime time) {
    prepare(time);
    return discretionaryChainsPerPerson;
  }

  /** Rainy days suppress the walking volume: fewer releases per time step. */
  @Override
  public double releaseBudgetMultiplier(LocalDateTime time) {
    if (!ActivityPars.useWeather) {
      return 1.0;
    }
    return Weather.isRainy(time.toLocalDate(), activity.seed())
        ? ActivityPars.rainReleaseMultiplier
        : 1.0;
  }

  /**
   * Persona-by-hour release affinity: commuter personas are favoured at the morning/evening peaks,
   * retirees at midday (see {@link pedsim.activity.agents.Persona#releaseAffinity}).
   */
  @Override
  public double releaseCandidateWeight(Agent agent, int hour) {
    if (!ActivityPars.usePersonaReleaseWeights
        || !(agent instanceof ActivityAgent activityAgent)
        || activityAgent.getPersona() == null) {
      return 1.0;
    }
    return activityAgent.getPersona().releaseAffinity(hour);
  }

  /**
   * Probability that a discretionary trip of this length is walked - the model's mode choice for
   * everything that is not a commute, asked once the destination is known.
   *
   * <p>Commuting is a different question and has its own curve; see
   * {@link #commuteWalkProbability(double, boolean)}.
   */
  @Override
  public double walkProbability(double meters) {
    return walkCurve(meters);
  }

  /**
   * The general walking-share logit, fitted to DfT NTS0308; see
   * {@link ActivityPars#walkShareHalfDistance} for what it imports and what it costs.
   */
  public static double walkCurve(double meters) {
    return 1.0
        / (1.0
            + Math.exp(
                ActivityPars.walkShareSteepness * (meters - ActivityPars.walkShareHalfDistance)));
  }

  @Override
  public double commuteWalkProbability(double meters, boolean student) {
    return commuteWalkCurve(meters, student);
  }

  /**
   * Probability that a commute of this length is walked. Fitted to ISTAT; the curve is a logit, and
   * there is one per purpose because the observed shares differ by more than a factor of two.
   *
   * <p>This is the model's only mode choice, and it is not optional: with it disabled every commute
   * in the model would be walked. It answers how a journey is made, which is a different question
   * from whether a given trip length is plausible - nothing filters trips by length.
   *
   * @param student whether this is a journey to a place of study
   */
  public static double commuteWalkCurve(double meters, boolean student) {
    double half =
        student
            ? ActivityPars.walkShareStudentHalfDistance
            : ActivityPars.walkShareCommuteHalfDistance;
    double steepness =
        student ? ActivityPars.walkShareStudentSteepness : ActivityPars.walkShareCommuteSteepness;
    return 1.0 / (1.0 + Math.exp(steepness * (meters - half)));
  }
}
