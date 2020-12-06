## PedSimCity: An Agent-Based Model for simulating pedestrian movement in large urban areas

This model incoporates simplified cognitive maps of the urban environment in the architecture of the agents
In particular, a computational approach to Kevyn Lynch's The Image of the City (see related paper in [Cities](https://www.sciencedirect.com/science/article/pii/S0264275118309776)) is employed to incorporate salient urban element in the route-choice approaches of the agents.

The model is built on Mason and [GeoMason 1.6](https://cs.gmu.edu/~eclab/projects/mason/extensions/geomason/).

A basic conceptual model of the ABM was presented at the [2018 Agile Conference](https://agile-online.org/conference_paper/cds/agile_2018/shortpapers/64%20short_paper_64.pdf).

The introduction of different urban elements has been tested, in combination with existing route choice models:
* Landmark-based navigation: London - [landmarkBased branch](https://github.com/g-filomena/pedSimCity/tree/LandmarkBased) - Methods, results and evaluation are documented in *Modelling the effect of landmarks on pedestrian dynamics* (in publication in *Computers, Environment and Urban Systems*).
* Region- and barrier-based navigation: London and Paris - [regionBased branch](https://github.com/g-filomena/pedSimCity/tree/RegionBased) - Methods and results, along with a validation are documented in *Perception of urban subdivisions in pedestrian movement simulation* (in publication in *PLoS ONE*).

It requires:
* GeoMason and its requirements;
* [UrbanSim](https://github.com/g-filomena/urbanSim), a set of supporting functions for urban simulation models.

Before running the model, check the default parameters in the class UserParameters.java so to run the ABM as you please; one can also set the directories for storing the output.
The core of the simulation (the simulation state, the scheduler and the agent basic movement functions) is based on *Gridlock*, by Sarah Wise, Mark Coletti, and Andrew Crooks, a GeoMason example.
