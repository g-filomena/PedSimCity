package sim.app.geo.pedSimCity;


import sim.app.geo.urbanSim.NodesLookup;

public class ActivityPlanner{


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
}


