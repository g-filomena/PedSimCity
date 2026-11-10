package pedsim.learning.cognitivemap;

import pedsim.learning.parameters.LearningPars;

/**
 * How a learner builds and keeps what it learns of a place: one of the three groups of Weisberg and
 * Newcombe's cognitive-map studies.
 *
 * <p>An {@link #INTEGRATOR} relates the routes it has walked into a map; a {@link #NON_INTEGRATOR}
 * knows each route but not how they connect; an {@link #IMPRECISE} navigator knows the routes less
 * well too. The group follows the agent's {@code spatialAbility}: the lowest {@code
 * LearningPars.impreciseShare} of the population are imprecise, the highest {@code
 * LearningPars.integratorShare} integrate, the rest do not.
 *
 * <p>The profile sets three things. How much a walk adds to what is known: the imprecise add {@code
 * LearningPars.impreciseRouteGain} of it. How much of a walk is kept as map-like knowledge, and how
 * fast that fades: those without integration keep {@code LearningPars.unintegratedKeep} of the
 * integrators' share and lose it {@code LearningPars.unintegratedFade} times as fast. What fades
 * fast, the walked traces, fades at one rate for everybody.
 */
public enum MemoryProfile {
  INTEGRATOR,
  NON_INTEGRATOR,
  IMPRECISE;

  /** The group of an agent with this spatial ability, in [0, 1]. */
  public static MemoryProfile of(double spatialAbility) {
    double imprecise =
        pedsim.core.cognition.cognitivemap.CognitiveMap.spatialAbilityAt(
            LearningPars.impreciseShare);
    double integrator =
        pedsim.core.cognition.cognitivemap.CognitiveMap.spatialAbilityAt(
            1.0 - LearningPars.integratorShare);
    if (spatialAbility < imprecise) {
      return IMPRECISE;
    }
    return spatialAbility >= integrator ? INTEGRATOR : NON_INTEGRATOR;
  }

  /** What a walk adds, as a multiple of the walks it counts for. */
  public double routeGain() {
    return this == IMPRECISE ? LearningPars.impreciseRouteGain : 1.0;
  }

  /** The multiple of {@code LearningPars.consolidatedShare} this group keeps of a walk. */
  public double keep() {
    return this == INTEGRATOR ? 1.0 : LearningPars.unintegratedKeep;
  }

  /** The multiple of the consolidated store's fading rate this group has. */
  public double fade() {
    return this == INTEGRATOR ? 1.0 : LearningPars.unintegratedFade;
  }
}
