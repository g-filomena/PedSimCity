## PedSimCity: An Agent-Based Model for simulating pedestrian movement in large urban areas

This model incorporates simplified cognitive maps of the urban environment in the architecture of the agents.
In particular, a computational approach to Kevyn Lynch's The Image of the City (see related paper in [Cities](https://www.sciencedirect.com/science/article/pii/S0264275118309776)) is employed to incorporate salient urban elements in the route choice approaches of the agents.
A basic conceptual model of the ABM was presented at the [2018 Agile Conference](https://agile-online.org/conference_paper/cds/agile_2018/shortpapers/64%20short_paper_64.pdf).

The introduction of different urban elements has been tested, in combination with existing route choice models:
* Landmark-based navigation: London - [landmarkBased branch](https://github.com/g-filomena/pedSimCity/tree/LandmarkBased) - Methods, results and evaluation are documented in *Modelling the effect of landmarks on pedestrian dynamics*, published in [Computers, Environment and Urban Systems](https://doi.org/10.1016/j.compenvurbsys.2020.101573).
* Region- and barrier-based navigation: London and Paris - [regionBased branch](https://github.com/g-filomena/pedSimCity/tree/RegionBased) - Methods and results, along with a validation are documented in *Perception of urban subdivisions in pedestrian movement simulation*, published in [PLoS ONE](https://doi.org/10.1371/journal.pone.0244099).

The role of the landmarks, barriers and region are combined and modelled together in the Empirical based Agent-Based Model (Master Branch).
The ABM, the qualitative study conducted to calibrate it, and its evaluation are documented in *Empirical characterisation of agents’ spatial behaviour in pedestrian movement simulation*, 
published in [Journal of Environmental Psychology](https://www.sciencedirect.com/science/article/pii/S0272494422000524).

The ABM is built on:
* [JTS](https://github.com/locationtech/jts)
* [Mason](https://cs.gmu.edu/~eclab/projects/mason/extensions/geomason/)
* [GeoMason-light](https://github.com/g-filomena/GeoMason-light)

Along with:
* [Apache Commons Lang](https://commons.apache.org/proper/commons-lang/download_lang.cgi)
* [OpenCsv](http://opencsv.sourceforge.net)
* [Java Tuples](https://www.javatuples.org)
* [SLF4J](https://www.slf4j.org)

How to run the applet:
1. Install Java on your machine.
2. Download the jar file *PedSimCity1.1-jar-with-dependencies.jar* wherever it is convenient.
3. Open the command prompt in the directory where the .jar file is placed.
4. Run the command *java -jar PedSimCity1.1-jar-with-dependencies.jar*;
5. The applet should pop-up and log-messages should appear in the command prompt window.

**This is the recommended option for running PedSimCity and it does not require the user to take any other step or to manually install the dependencies.**

If the user desires to use the applet within Eclipse, for example, to explore the source files or to make changes, the following instructions should be followed:

1. Download the raw content of the Github PedSimCity Repository, as a .zip file.
2. Unzip the file and move the nested PedSimCity-Master folder wherever it is convenient. 
3. Open Eclipse, and create a new Java project; any name will do.
4. Right click on the project on the left-hand side *Package Explorer*. Select *Build Path*, *Link Source*, navigate to the PedSimCity-Master, navigate to and then select the folder *src/main/java* (without double clicking on it).
4. Import all the libraries mentioned above, manually, by right clicking on your project *Build Path*, *Add External Archives*.
5. To execute the applet, right-click on teh class ```PedSimCity.applet```, *Run as Java Application*
6. Before pressing the *Run Simulation* button, click on *Other options* and copy-paste the entire path referring to the path *src/main/resources/* in the corresponding field.


Instruction for running the ABM as is for the included case study areas: Paris (regions and barriers), London (landmarks, regions, barriers) or Muenster (landmarks, regions, barriers, empirical-based).
1. Open the class *UserParameters.java* and get familiar with it.
2. Specify the following parameter values, so to run the ABM as desired: e.g. number of ```jobs```, ```numAgents``` per scenario, ```numTrips``` per agent (these values are pre-defined when modelling the effect of landmarks and urban subdivisions).
2. Set the ```cityName```.
2. Set the local directories for storing the simulation output (this is important).
* Choose whether to model the effect of landmarks (```testingLandmarks```), regions and barriers (```testingRegions```). These settings run the ABM with pre-defined set of scenarios (and route choice models assigned to each scenario) as described in the corresponding papers.
* Alternatively one can choose to define their own scenarios. To do so, ```testingModels``` needs to be ```True``` and the Array ```routeChoices``` needs to be filled with the desired route choice models (see the *UserParameters* class for details).
* To run the empirical based agent-based model, ```empiricalABM``` should be set to ```True```. If ```usingDMA``` is ```True```, the model will also select destination nodes on the basis of empirical data regarding walking trip purposes (see the paper for details).

Other aspects to note:
* ```testingSpecificRoutes``` allows the user to prevent the model to define the Origin-Destination (OD) Matrix. In this case the user can input the desired OD pairs manually in the lists ```or``` and ```de``` (using the corresponding *nodeID* of the junctions).
* One may have to define the number of ABM runs, the agents per scenario, as well as the number of trips per agent. 
* ```testingLandmarks```, ```testingRegions```, ```testingModels```, and ```empiricalABM``` are mutually exclusive. However, they can all be combined with ```testingSpecificRoutes```
* Check that the route choice models to be included refer to elements for which the case study's data is available (e.g. do not model landmarks for Paris).

The core of the simulation (the simulation state, the scheduler and the agent basic movement functions) is based on *Gridlock*, by Sarah Wise, Mark Coletti, and Andrew Crooks, a GeoMason example.
