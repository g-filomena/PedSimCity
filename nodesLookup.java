package sim.app.geo.pedestrianSimulation;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.Iterator;
import java.util.List;
import java.util.Map;

import com.vividsolutions.jts.geom.Coordinate;
import com.vividsolutions.jts.geom.Geometry;
import com.vividsolutions.jts.geom.GeometryFactory;
import com.vividsolutions.jts.planargraph.Node;
import sim.field.geo.GeomVectorField;
import sim.util.Bag;
import sim.util.geo.MasonGeometry;

public class nodesLookup {
	
    
    static Node searchRandomNode(Bag geometriesNodes, pedestrianSimulation state)
    {
    	int c = state.random.nextInt(geometriesNodes.size());  	
    	MasonGeometry geoNode = (MasonGeometry) geometriesNodes.objs[c]; 
    	return state.network.findNode(geoNode.geometry.getCoordinate());
    }

    static Node searchNodeWithin(Node originNode, GeomVectorField vectorField, List<Float> distances, pedestrianSimulation state)
    {
		GeometryFactory fact = new GeometryFactory();
    	MasonGeometry nodeLocation = new MasonGeometry(fact.createPoint(new Coordinate(originNode.getCoordinate())));
    	Geometry geo = nodeLocation.geometry;
    	int pD = state.random.nextInt(distances.size());
    	Float distance = distances.get(pD);
    	Node node = null;
    	Bag filter = null;
    	double range = 5;
    	
    	while(true)
    	{
	 	  	double lowL = distance - distance*range;
	 	    double uppL = distance + distance*range;
	 	  	filter = vectorField.getWithinObjects(geo, lowL, uppL);	 
	 	  	if (filter.size() > 1) break;
	 	  	else range = range + 5;
    	}
    	
    	MasonGeometry geoNode = null;
    	while (geoNode == null || (node.getData() == originNode.getData()))
    	{
	 	  	int c = state.random.nextInt(filter.size());
	 	  	geoNode = (MasonGeometry) filter.objs[c];
	 	  	node = state.network.findNode(geoNode.geometry.getCoordinate());
    	}
 	  	return node;
    }
    
    static Node searchNodeWithin(Geometry geo, GeomVectorField vectorField, double lowL, double uppL, pedestrianSimulation state)
    {	    
 	  	Bag filter = vectorField.getWithinObjects(geo, lowL, uppL);	  	
 	  	int c = state.random.nextInt(filter.size());
 	  	MasonGeometry geoNode = (MasonGeometry) filter.objs[c];
 	  	return state.network.findNode(geoNode.geometry.getCoordinate());
    }
    
    static Node searchNodeDistributionWithin(Geometry geo, GeomVectorField vectorField,  
    		HashMap<MasonGeometry, Double> nodesMap, double lowL, double uppL, pedestrianSimulation state)
    {
		double p = state.random.nextFloat();
	  	MasonGeometry  randomNode = null;
        HashMap<MasonGeometry, Double> nodesTmpMap  = new HashMap<MasonGeometry, Double> (nodesMap);	  	
        ArrayList<MasonGeometry> result = new ArrayList<MasonGeometry>();
        
	  	Bag filter = vectorField.getWithinObjects(geo, lowL, uppL);	  
        for(MasonGeometry key : nodesTmpMap.keySet()) {if(filter.contains(key)) {result.add(key);}}
        
        nodesTmpMap.keySet().retainAll(result);
        double sumMetric =  nodesTmpMap.values().stream().mapToDouble(d->d).sum();
        
        // computing probabilities
        for (MasonGeometry key : nodesTmpMap.keySet())
        {
        	double rc = nodesTmpMap.get(key);
        	double probRC = rc/sumMetric;
        	nodesTmpMap.put(key, probRC);
        }
       
        Map orderedNodes = utilities.sortByValue(nodesTmpMap); 	
 	  	Iterator it = orderedNodes.entrySet().iterator();
 	  	
 	  	while (it.hasNext()) 
 	  	{
 	  	   double cumulative = 0.00;
 	       Map.Entry<MasonGeometry, Double> entry = (Map.Entry<MasonGeometry, Double>)it.next();
 	       double probNode = entry.getValue();
 	       cumulative += probNode;
 	        if (cumulative < p) continue;
 	        if (cumulative >= p) 
        	{
	        	randomNode = entry.getKey();
	        	break;
        	}
 	    }
 	  	return state.network.findNode(randomNode.geometry.getCoordinate());
    }
    
   
    Node searchNodeDistribution(pedestrianSimulation state, HashMap<MasonGeometry, Double> nodesMap)
    {
    	double p = state.random.nextFloat();
    	double cumulative = 0.00;
 	  	MasonGeometry randomNode = null; 	
        Map orderedNodes = utilities.sortByValue(nodesMap); 	
 	  	Iterator it = orderedNodes.entrySet().iterator();

 	  	while (it.hasNext()) 
 	  	{
 	       Map.Entry<MasonGeometry, Double> entry = (Map.Entry<MasonGeometry, Double>) it.next();
 	       double probNode = entry.getValue();
 	       cumulative += probNode;
 	        if (cumulative < p) continue;
 	        if (cumulative >= p) 
 	        {
 	        	randomNode = entry.getKey();
 	        	break;
 	        }
 	    }
 	  	return state.network.findNode(randomNode.geometry.getCoordinate());
    }

}
