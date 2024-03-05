# PedSimCity #

## An Agent-Based Model for simulating pedestrian movement in large urban areas ##

[![GitHub CI](https://github.com/g-filomena/PedSimCity/actions/workflows/build.yaml/badge.svg)](https://github.com/g-filomena/PedSimCity/actions/workflows/build.yaml)
[![License](https://img.shields.io/badge/License-GPLv3-blue.svg)](https://www.gnu.org/licenses/gpl-3.0.en.html)

This model simulates the movement of pedestrians across the street network of large urban areas. The novelty of the model lies on the inclusion of cognitive representations of space (cognitive maps) in the behavioural architecture of the pedestrian agents.

More specifically, a computational approach to Kevin Lynch's The Image of the City (see related paper in [Cities](https://www.sciencedirect.com/science/article/pii/S0264275118309776)) is employed to incorporate salient urban elements in the cognitive maps of the agents - alongside perception of distances and angular relationships between road segments. 
It is argued that the include of certain urban elements in one’s cognitive map shapes their route choice behaviour, that is how they formulate a route between an origin and a destination. 

The ABM has been built following a stepwise approach, so as to explore and assess the effect of the inclusion in the cognitive map of the agents of different urban elements (1. landmarks, 2. regions and barriers). 

The impact of element-based route choice models within the model was assessed in comparison with minimisation-based route choice models (i.e. distance shortest path, least cumulative angular change). 
The inclusion of these urban elements has been tested, in combination with existing route choice models:
* Landmark-based navigation: London - Methods, results and evaluation are documented in *Modelling the effect of landmarks on pedestrian dynamics*, published in [Computers, Environment and Urban Systems](https://doi.org/10.1016/j.compenvurbsys.2020.101573).
* Region- and barrier-based navigation: London and Paris - Methods and results, along with a validation are documented in *Perception of urban subdivisions in pedestrian movement simulation*, published in [PLoS ONE](https://doi.org/10.1371/journal.pone.0244099).

The ABM allows executing these experiments *Testing Landmarks* and *Testing Urban Subdivisions*.

In addition, the ABM, can be run as an empirical-based model where the interaction between the effects of the different urban elements is regulated and calibrated on the basis of empirical data (“Empirical ABM”). 
The ABM, the qualitative study conducted to calibrate it, and its evaluation are documented in *Empirical characterisation of agents’ spatial behaviour in pedestrian movement simulation*, 
published in [Journal of Environmental Psychology](https://www.sciencedirect.com/science/article/pii/S0272494422000524).

![](figures/urbanSub.png)
*Testing Urban Subdivisions: Pedestrian Volumes after elaboration in Python (London, UK)*

PedSimCity is built on:
* [JTS](https://github.com/locationtech/jts)
* [Mason](https://cs.gmu.edu/~eclab/projects/mason/extensions/geomason/)
* [GeoMason-light](https://github.com/g-filomena/GeoMason-light)

Along with:
* [Apache Commons Lang](https://commons.apache.org/proper/commons-lang/download_lang.cgi)
* [OpenCsv](http://opencsv.sourceforge.net)
* [Java Tuples](https://www.javatuples.org)
* [SLF4J](https://www.slf4j.org)

**How to run the applet:**
1. Install Java on your machine.
2. Download the jar file *pedsimcity1.23-jar-with-dependencies.jar* wherever it is convenient.
3. Open the command prompt in the directory where the .jar file is placed.
4. Run the command *java -jar pedsimcity1.23-jar-with-dependencies.jar*.
5. The applet should pop-up and log-messages should appear in the command prompt window.

**This is the recommended option for running PedSimCity and it does not require the user to take any other step or to manually install the dependencies.**

If the user desires to use the applet within Eclipse, for example, to explore the source files or to make changes, the following instructions should be followed:

1. Download the raw content of the Github `PedSimCity` Repository, as a .zip file.
2. Unzip the file and move the nested PedSimCity-Master folder wherever it is convenient. 
3. Open Eclipse, and create a new Java project; any name will do.
4. Right click on the project on the left-hand side *Package Explorer*. Select *Build Path*, *Link Source*, navigate to the PedSimCity-Master, navigate to and then select the folder *src/main/java* (without double clicking on it).
4. Import all the libraries mentioned above, manually, by right clicking on your project *Build Path*, *Add External Archives*.
5. To execute the applet, right-click on teh class ```PedSimCity.applet```, *Run as Java Application*.
6. Before pressing the *Run Simulation* button, click on *Other options* and copy-paste the entire path referring to the path *src/main/resources/* in the corresponding field. This is necessary for retrieving the input data.

The applet allows the user to run the simulation with three different configurations:
1. Testing Landmarks (London, Muenster).
2. Testing Urban Subdivisions (London, Paris, Muenster).
3. Testing Specific Route Choice Models (Muenster).
4. Empirical ABM (Muenster).

Options 1, 2 and 4 all come with pre-defined set as regards the parameters: number of ```jobs```, ```numAgents``` per scenario, ```numberTripsPerAgent```. This is line with the settings used for producing the results presented in the papers mentioned above.
When ```testingLandmarks``` and  ```testingSubdivisions```, the user can however runs the model for specific ODs by checking the ```Testing Specific ODs``` box and inputing the nodeIDs in the corresponding fields (the number of ```jobs``` won't change).
The user can also change other simulation-related parameters by clicking on the ```Other Options``` button, before starting the simulation. 

When choosing option 3, the route choice models of interest need to be chosen by clicking the ```Choose Route Choices``` button. 
The user can also define the number of ```jobs```, and ```numberTripsPerAgent``` (one route choice model = one agent).
