package sim.app.geo.PedSimCity;


import java.util.Random;

import sim.app.geo.UrbanSim.NodesLookup;
import sim.engine.SimState;

/**
 * This class contains functions for scheduling different activities when an entire day is simulate UserParameters.activityPlanner;
 */
public class ActivityPlanner{
	Pedestrian ped;

	public void goHome(Pedestrian ped) {
		ped.ap.away = false;
		ped.originNode = ped.destinationNode;
		ped.ap.setThresholdHome();
		ped.destinationNode = ped.ap.homePlace;
		ped.ap.totalTimeAway = 0.0;
		go(ped);
	}

	public void goForLeisure(Pedestrian ped) {
		ped.originNode = ped.destinationNode;
		ped.destinationNode = NodesLookup.randomNode(PedSimCity.network, PedSimCity.junctions.geometriesList);
		ped.ap.otherPlace = ped.destinationNode;
		ped.ap.setThresholdAway("leisure");
		go(ped);
	}

	public void goToWork(Pedestrian ped) {
		// work
		ped.originNode = ped.destinationNode;
		ped.ap.setThresholdAway("work");
		ped.destinationNode = ped.ap.workPlace;
		go(ped);
	}

	public void goForErrands(Pedestrian ped) {
		// work
		ped.originNode = ped.destinationNode;
		ped.destinationNode = NodesLookup.randomNode(PedSimCity.network, PedSimCity.junctions.geometriesList);
		ped.ap.otherPlace = ped.destinationNode;
		ped.ap.setThresholdAway("errand");
		go(ped);
	}

	private void go(Pedestrian ped)
	{
		ped.ap.setAway();
		ped.findNewAStarPath(ped.state);
	}

	public void checkRoutine(SimState state, Pedestrian ped) {

		double minutesSoFar = ped.minutesSoFar;

		ped.ap.totalTimeAway += UserParameters.minutesPerStep;
		if (ped.reachedDestination) {
			if (ped.destinationNode == ped.ap.workPlace) ped.ap.atWork = true;
			if (ped.destinationNode == ped.ap.homePlace) ped.ap.atHome = true;
			if (ped.destinationNode == ped.ap.otherPlace) ped.ap.away = true;
			ped.ap.atPlace = true;
			ped.reachedDestination = false;
		}
		if (!ped.reachedDestination & !ped.ap.atPlace) return;

		if (ped.ap.atWork | ped.ap.away) ped.ap.timeAway += UserParameters.minutesPerStep;
		else if (ped.ap.atHome) ped.ap.timeAtHome += UserParameters.minutesPerStep;
		if ((ped.ap.timeAtWork < ped.ap.thresholdAway) | (ped.ap.timeAtHome < ped.ap.thresholdAtHome) | (ped.ap.timeAway < ped.ap.thresholdAway)) return;

		// new activity
		double probNightOut = 0.40;
		double probErrands = 0.35;
		Random random = new Random();

		// 20.00 pm (approx)
		int evening = 20*60+random.nextInt(30);
		int afternoon = 15*60+random.nextInt(30);
		int morning = 8*60+random.nextInt(30);

		if (ped.ap.student || ped.ap.worker)
		{
			if (ped.ap.atWork) {
				// either go home or for errands (25% of chances if it's earlier than 20pm
				if (random.nextFloat() <= probErrands & minutesSoFar < evening) this.goForErrands(ped);
				else this.goHome(ped);
			}
			else if (ped.ap.atHome & ped.ap.timeAtWork == 0.0) goToWork(ped);
			else if (ped.ap.atHome & ped.ap.timeAtWork > 0.0) {
				if (ped.ap.student) probNightOut = 0.60;
				// go out or for other errands
				if (minutesSoFar < evening & random.nextFloat() <= probErrands) goForErrands(ped);
				else if (minutesSoFar > evening & random.nextFloat() <= probNightOut) goForLeisure(ped);
			}
			else if (ped.ap.away) goHome(ped);
		}
		else if (ped.ap.flaneur) {
			if ((ped.ap.atHome && minutesSoFar > morning) || (ped.ap.totalTimeAway < ped.ap.thresholdWandering)) {
				if (minutesSoFar < evening & random.nextFloat() <= probErrands) goForErrands(ped);
				goForLeisure(ped);
			}
			else if (ped.ap.totalTimeAway >= ped.ap.thresholdWandering) goHome(ped);
		}
		else if (ped.ap.homeBased) {
			if (ped.ap.atHome && minutesSoFar < evening) {
				// either go home or for errands (45% of chances if it's earlier than 20pm
				if (minutesSoFar > morning & random.nextFloat() <= probErrands) goForErrands(ped);
				else if (minutesSoFar > afternoon) goForLeisure(ped);
			}
			// either go home or for errands (25% of chances if it's earlier than 20pm
			else if (ped.ap.atHome && minutesSoFar > evening && random.nextFloat() <= probNightOut) goForLeisure(ped);
			else if (ped.ap.away) goHome(ped);
		}
		return;
	}
}


