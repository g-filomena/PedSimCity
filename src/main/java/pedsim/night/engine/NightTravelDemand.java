package pedsim.night.engine;

import pedsim.activity.engine.ActivityTravelDemand;
import pedsim.core.agents.Agent;
import pedsim.core.parameters.TimePars;
import pedsim.core.utilities.LoggerUtil;
import pedsim.night.agents.NightAgent;
import pedsim.night.parameters.NightPars;

/** Night travel demand: the activity tier's, plus the light A/B experiment's paired releases. */
public class NightTravelDemand extends ActivityTravelDemand {

  private final PedSimCityNight night;

  public NightTravelDemand(PedSimCityNight state) {
    super(state);
    this.night = state;
  }

  /**
   * With light A/B testing enabled, day 1 releases one vulnerable/non-vulnerable twin pair per
   * release event (agents {@code 2i} and {@code 2i+1} for release event {@code i}, 72 pairs)
   * instead of the standard release.
   */
  @Override
  public int releaseAgentsOverride(double steps, int dayNumber) {
    if (!NightPars.enableLightABTesting || dayNumber != 1) {
      return -1;
    }

    // One pair per release event, in the order NightPopulate built them. The bound is the number
    // of pairs that actually exist: it was hardcoded to 72, so raising abTestPairs left the extra
    // pairs at home for the whole day, and lowering it sent every later event scanning the agent
    // list for twins that were never built.
    int pairIndex = (int) Math.round(steps / TimePars.releaseAgentsEverySteps) - 1;
    if (pairIndex < 0 || pairIndex >= Math.max(1, NightPars.abTestPairs)) {
      return 0;
    }

    NightAgent vulnAgent = null;
    NightAgent normalAgent = null;
    for (Agent agent : night.agentsList) {
      if (agent instanceof NightAgent nightAgent) {
        if (nightAgent.agentID == pairIndex * 2) {
          vulnAgent = nightAgent;
        } else if (nightAgent.agentID == pairIndex * 2 + 1) {
          normalAgent = nightAgent;
        }
      }
    }
    if (vulnAgent == null || normalAgent == null) {
      return 0;
    }

    // The pair must differ in vulnerability and in nothing else. What makes that true is the
    // shared destination: NightAgent.defineRandomDestination copies its twin's, whichever of the
    // two chooses first. A shared trip *length* was handed to both here as well, drawn from a band
    // of metres, and it was the last trip-distance parameter left in the activity tier - which is
    // the tier that is not supposed to have one, night included. It bought nothing the shared
    // destination does not already buy.
    vulnAgent.startWalkingAlone();
    normalAgent.startWalkingAlone();

    LoggerUtil.getLogger()
        .fine(
            "A/B Testing: Released pair "
                + pairIndex
                + " (Agent "
                + vulnAgent.agentID
                + " & "
                + normalAgent.agentID
                + ") at step "
                + steps);
    return 2;
  }

}
