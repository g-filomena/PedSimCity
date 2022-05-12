## PedSimCity: An Agent-Based Model for simulating pedestrian movement in large urban areas

This model incoporates simplified cognitive maps of the urban environment in the architecture of the agents
In particular, a computational approach to Kevyn Lynch's The Image of the City (see related paper in [Cities](https://www.sciencedirect.com/science/article/pii/S0264275118309776)) is employed to incorporate salient urban element in the route-choice approaches of the agents.

The model is built on Mason and [GeoMason 1.6](https://cs.gmu.edu/~eclab/projects/mason/extensions/geomason/).

A basic conceptual model of the ABM was presented at the [2018 Agile Conference](https://agile-online.org/conference_paper/cds/agile_2018/shortpapers/64%20short_paper_64.pdf).

The introduction of different urban elements has been tested, in combination with existing route choice models:
* Landmark-based navigation: London - [landmarkBased branch](https://github.com/g-filomena/pedSimCity/tree/LandmarkBased) - Methods, results and evaluation are documented in *Modelling the effect of landmarks on pedestrian dynamics*, published in [Computers, Environment and Urban Systems](https://doi.org/10.1016/j.compenvurbsys.2020.101573).
* Region- and barrier-based navigation: London and Paris - [regionBased branch](https://github.com/g-filomena/pedSimCity/tree/RegionBased) - Methods and results, along with a validation are documented in *Perception of urban subdivisions in pedestrian movement simulation*, published in [PLoS ONE](https://doi.org/10.1371/journal.pone.0244099).

The role of the landmarks, barriers and region in combined and modelled toghether in the Empirical based Agent-Based Model (Master Branch). The ABM, the qualitative study conducted to calibrate it, and its evaluation are documented in *Empirical characterisation of agents’ spatial behaviour in pedestrian movement simulation*, published in [Journal of Environmental Psychology](https://www.sciencedirect.com/science/article/pii/S0272494422000524).


The ABM is built on and it requires:
* [JTS] (https://github.com/locationtech/jts)
* [Mason and GeoMason](https://cs.gmu.edu/~eclab/projects/mason/extensions/geomason/)

Along with:
* [Apache Commons Lang](https://commons.apache.org/proper/commons-lang/download_lang.cgi)
* [OpenCsv](http://opencsv.sourceforge.net)
* [Java Tuples](https://www.javatuples.org)

Instructions for using it in Eclipse:
1. Create a new project in Eclipse and give it a name.
2. Download *pedsimcity*, unzip it and place it wherever it is convenient. Rename the main folder as *pedsimcity* if it is not names so already
3. Right click on your project on the left-hand side Package Explorer. Select *Build Path*, *Link Source* and then just select the folder *pedsimcity*.
4. For all the libraries, download the JAR files and install them in the sequence described above by right clicking on your project *Build Path*, *Add External Archives*.

If you do not like to have Mason and Geomason installed from the JAR files, which would include all the example models too and which would not allow you to edit and have a look at their source code, you can download them from 



Instruction for running the model as is for the included case study areas: Paris, London or Muenster
Before running the model, check the default parameters in the class UserParameters.java so to run the ABM as you please; there, one can also set the local directories for storing the output.



The core of the simulation (the simulation state, the scheduler and the agent basic movement functions) is based on *Gridlock*, by Sarah Wise, Mark Coletti, and Andrew Crooks, a GeoMason example.
