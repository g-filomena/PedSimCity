/**
 ** GridlockWithUI.java
 **
 ** Copyright 2011 by Sarah Wise, Mark Coletti, Andrew Crooks, and
 ** George Mason University.
 **
 ** Licensed under the Academic Free License version 3.0
 **
 ** See the file "LICENSE" for more information
 *
 * $Id: GridlockWithUI.java 842 2012-12-18 01:09:18Z mcoletti $
 **
 **/
package sim.app.geo.pedSimCity;

import java.awt.Color;
import java.awt.Font;
import java.awt.Graphics2D;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.IOException;
import java.net.URL;
import java.util.Arrays;

import javax.swing.JFrame;
import org.jfree.data.xy.XYSeries;

import com.opencsv.CSVReader;

import sim.display.Console;
import sim.display.Controller;
import sim.display.Display2D;
import sim.display.GUIState;
import sim.engine.SimState;
import sim.engine.Steppable;
import sim.io.geo.ShapeFileImporter;
import sim.portrayal.DrawInfo2D;
import sim.portrayal.FieldPortrayal2D;
import sim.portrayal.geo.GeomPortrayal;
import sim.portrayal.geo.GeomVectorFieldPortrayal;
import sim.util.Bag;
import sim.util.geo.MasonGeometry;
import sim.util.media.chart.TimeSeriesChartGenerator;



public class PedSimCityUI extends GUIState
{

    public Display2D display;
    public JFrame displayFrame;
    private GeomVectorFieldPortrayal roadsPortrayal = new GeomVectorFieldPortrayal(true);
    private GeomVectorFieldPortrayal tractsPortrayal = new GeomVectorFieldPortrayal(true);
    private GeomVectorFieldPortrayal junctionsPortrayal = new GeomVectorFieldPortrayal(true);
    private GeomVectorFieldPortrayal agentPortrayal = new GeomVectorFieldPortrayal();
    
    TimeSeriesChartGenerator trafficChart;
    XYSeries maxSpeed;
    XYSeries avgSpeed;
    XYSeries minSpeed;
    double ratio;

    protected PedSimCityUI(SimState state)
    {
        super(state);
    }

    /**
     * Main function
     * @param args
     */
    public static void main(String[] args)
    {
    	String directory = "C:/Users/g_filo01/sciebo/Scripts/Image of the City/Outputs/London/intermediate/for Simulation/";
        try 
        {
	    	///// READING PRIMAL GRAPH
	    	System.out.println("reading primal graph...");
	
            Bag junctionsAttributes = new Bag();
            Bag buildingsAttributes = new Bag();
   
            junctionsAttributes.add("nodeID"); 
	        junctionsAttributes.add("loc_land"); 
	        junctionsAttributes.add("loc_scor"); 
	        junctionsAttributes.add("dist_land"); 
	        junctionsAttributes.add("dist_scor"); 
	        junctionsAttributes.add("anchors");   
	        junctionsAttributes.add("dist_anch");      
	        junctionsAttributes.add("distances");  
	        junctionsAttributes.add("Bc_E"); 

	        
	        if (PedSimCity.districtRouting)
			{
		        junctionsAttributes.add("Bc_multi");
		        junctionsAttributes.add("nodeID"); 
		        junctionsAttributes.add("district"); 
		        junctionsAttributes.add("regionsTo"); 
		        junctionsAttributes.add("CM"); 
			}
	        
	        URL roadsFile = PedSimCity.class.getResource("data/London_edges_simplified_BB.shp");
	        URL junctionsFile = PedSimCity.class.getResource("data/London_nodes_simplified_BB.shp");
	        ShapeFileImporter.read(roadsFile, PedSimCity.roads);
	        ShapeFileImporter.read(junctionsFile, PedSimCity.junctions, junctionsAttributes);
	        
	        ///// READING DUAL GRAPH ------------
	        System.out.println("reading dual graph...");
	        Bag centroidsAttributes = new Bag();
	        centroidsAttributes.add("edgeID");
	        URL roadsDualFile = PedSimCity.class.getResource("data/London_edgesDual_simplified_BB.shp");
	        URL centroidsFile = PedSimCity.class.getResource("data/London_nodesDual_simplified_BB.shp");            
	        ShapeFileImporter.read(roadsDualFile, PedSimCity.intersectionsDual);
	        ShapeFileImporter.read(centroidsFile, PedSimCity.centroids, centroidsAttributes);
	
	        
	        ///// READING BUILDINGS
	        System.out.println("reading buildings layer...");
	        buildingsAttributes.add("buildingID");  
	        buildingsAttributes.add("lScore_sc");  
	        buildingsAttributes.add("gScore_sc");  
	        URL landmarksFile = PedSimCity.class.getResource("data/London_landmarks.shp");
	        ShapeFileImporter.read(landmarksFile, PedSimCity.buildings);
	
	        /// READING DISTANCES
			System.out.println("reading distances...");
			CSVReader readerDistances = new CSVReader(new FileReader
			  		("C:/Users/g_filo01/sciebo/Scripts/GPS Trajectories/Outputs/London/GPStracks/London_traj_distances.csv"));
			String[] nextLineDistances;
			  
			int ds = 0;
			while ((nextLineDistances = readerDistances.readNext()) != null) 
			{
			  ds += 1;
			  if (ds == 1) continue;
			  PedSimCity.distances.add(Float.parseFloat(nextLineDistances[2]));
			}
			readerDistances.close();
	        
			PedSimCity.network.createFromGeomField(PedSimCity.roads);
			PedSimCity.dualNetwork.createFromGeomField(PedSimCity.intersectionsDual);          			
			PedSimCity.nodesGeometries = PedSimCity.junctions.getGeometries();
			PedSimCity.centroidsGeometries = PedSimCity.centroids.getGeometries();
	        
	        if (PedSimCity.visibility)
	        {
	            System.out.println("reading visibility...");
	            PedSimCity.visibilityMatrix = new String[PedSimCity.buildings.getGeometries().size()+1][PedSimCity.nodesGeometries.size()+1];
	            CSVReader reader = new CSVReader(new FileReader(directory+"London_visibility_matrix_simplified_BB.csv"));
	            String [] nextLine;
	            
	            int v = 0;
	            while ((nextLine = reader.readNext()) != null) 
	            {
	            	PedSimCity.visibilityMatrix[v] = nextLine;
	            	v = v+1;
	            }
	            reader.close();
	        }
	        System.out.println("files imported successfully");
    	
	        PedSimCityUI simple = new PedSimCityUI(new PedSimCity(System.currentTimeMillis(), 1));
	        Console c = new Console(simple);
	        c.setVisible(true);
        }
        
        catch (IOException e)
        {
			e.printStackTrace();
		}
    	System.exit(0);
    }



    /**
     * @return name of the simulation
     */
    public static String getName()
    {
        return "Pedestrian Simulaiton in Urban Environments";
    }



    /**
     *  This must be included to have model tab, which allows mid-simulation
     *  modification of the coefficients
     */
    public Object getSimulationInspectedObject()
    {
        return state;
    }  // non-volatile



    /**
     * Called when starting a new run of the simulation. Sets up the portrayals
     * and chart data.
     */
    public void start()
    {
        super.start();
        PedSimCity world = (PedSimCity) state;

        maxSpeed = new XYSeries("Max Speed");
        avgSpeed = new XYSeries("Average Speed");
        minSpeed = new XYSeries("Min Speed");
        trafficChart.removeAllSeries();
        trafficChart.addSeries(maxSpeed, null);
        trafficChart.addSeries(avgSpeed, null);
        trafficChart.addSeries(minSpeed, null);
        
        state.schedule.scheduleRepeating(new Steppable()
        {

            public void step(SimState state)
            {
            	PedSimCity stateSchedule = (PedSimCity) state;
                double maxS = 0, minS = 10000, avgS = 0, count = 0;
                for (Pedestrian a : stateSchedule.agentList)
                {
                    if (a.reachedDestination) {continue;}
                    count++;
                    double speed = Math.abs(a.speed);
                    avgS += speed;
                    if (speed > maxS) {maxS = speed;}
                    if (speed < minS) {minS = speed;}
                    
                }
                double time = state.schedule.time();
                avgS /= count;
                maxSpeed.add(time, maxS, true);
                minSpeed.add(time, minS, true);
                avgSpeed.add(time, avgS, true);
            }

        });

        roadsPortrayal.setField(world.roads);
        roadsPortrayal.setPortrayalForAll(new GeomPortrayal(Color.DARK_GRAY, 0.002, false));
                
//      junctionsPortrayal.setField(world.junctions);
//      junctionsPortrayal.setPortrayalForAll(new GeomPortrayal(Color.BLUE, 10, true));
       
        agentPortrayal.setField(world.agents);
        agentPortrayal.setPortrayalForAll(new  GeomPortrayal()
        	{
        	public void draw(Object object, Graphics2D graphics, DrawInfo2D info) 
        		{
        		MasonGeometry p = (MasonGeometry) object;
        		Pedestrian agent = (Pedestrian) p.getUserData();
        		
          		String cr = agent.criteria;
        		
            	if (cr == "euclidean" || cr == "euclideanLand") paint = Color.RED;  
            	else if (cr == "angular" || cr == "angularLand") paint = Color.ORANGE;
            	else if (cr == "topological" || cr == "topologicalLand") paint = Color.GREEN;
            	else paint = Color.BLUE; //only landmark
            	scale = 10;
            	filled = true;
            	
        		super.draw(object, graphics, info);
        	} 
        });
        
        FieldPortrayal2D overlay = new FieldPortrayal2D()
        {
        Font font = new Font("SansSerif", 0, 18);  // keep it around for efficiency
        public void draw(Object object, Graphics2D graphics, DrawInfo2D info)
            {
            String s = "";
            if (state !=null)
                s = state.schedule.getTimestamp("Before Simulation", "All Done!");
            graphics.setColor(Color.blue);
            graphics.drawString(s, (int)info.clip.x + 10, 
                    (int)(info.clip.y + 10 + font.getStringBounds(s,graphics.getFontRenderContext()).getHeight()));

            }
        };
        
        display.reset();
        display.setBackdrop(Color.WHITE);
        display.repaint();

    }



    /**
     * Called when first beginning a WaterWorldWithUI. Sets up the display window,
     * the JFrames, and the chart structure.
     */
    public void init(Controller c)
    {
        super.init(c);

        // make the displayer
        display = new Display2D(1200, 1200*0.77, this);
        // turn off clipping
        // display.setClipping(false);

        displayFrame = display.createFrame();
        displayFrame.setTitle("Gridlock Display");
        c.registerFrame(displayFrame); // register the frame so it appears in
        // the "Display" list
        displayFrame.setVisible(true);

//        display.attach(tractsPortrayal, "Census Tracts");
        display.attach(roadsPortrayal, "Roads");
        display.attach(agentPortrayal, "Agents");
        display.attach(junctionsPortrayal, "Junctions");

        // CHART
        trafficChart = new TimeSeriesChartGenerator();
        trafficChart.setTitle("Traffic Statistics");
        trafficChart.setYAxisLabel("Speed");
        trafficChart.setXAxisLabel("Time");
        JFrame chartFrame = trafficChart.createFrame(this);
        chartFrame.pack();
        c.registerFrame(chartFrame);

    }
    

    /**
     * called when quitting a simulation. Does appropriate garbage collection.
     **/
    public void quit() 
    {	
    	super.quit();

        if (displayFrame != null)
        {
            displayFrame.dispose();
        }
        displayFrame = null; // let gc
        display = null; // let gc
    }

}