## PedSimCity: An Agent-Based Model for simulating pedestrian movement in large urban areas

This model incoporates simplified cognitive maps of the urban environment in the architecture of the agents
In particular, a computational approach to Kevyn Lynch's The Image of the City (see related [paper](https://www.sciencedirect.com/science/article/pii/S0264275118309776) in Cities) is employed to incorporate salient urban element in the route-choice approaches of the agents.

The model is built on Mason and [GeoMason](https://cs.gmu.edu/~eclab/projects/mason/extensions/geomason/)

A basic conceptual model of the ABM was presented at the Agile Conference, [short paper](https://agile-online.org/conference_paper/cds/agile_2018/shortpapers/64%20short_paper_64.pdf)
The introduction of different urban elements has been tested, in combination with existing route-choice models:
* Landmark-based navigation: London - [landmarkBased branch](https://github.com/g-filomena/pedSimCity/tree/LandmarkBased) - Methods, results and evaluation are documented in *Modelling Modelling the effect of landmarks on pedestrian dynamics* (in review, Computer, Environmenta and Urban Systems).
* Region- and barrier-based navigation: London and Paris - [regionBased branch]() - Methods and results, along with a validation are documented in *Perception of urban subdivisions in pedestrian movement simulation* (in review, PlosONE).


It requires:
* GeoMason and its requirements 
* [UrbanSim](https://github.com/g-filomena/urbanSim), a set of supporting functions