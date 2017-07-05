/**
 * @author UCSD MOOC development team and YOU
 * 
 * A class which reprsents a graph of geographic locations
 * Nodes in the graph are intersections between 
 *
 */
package roadgraph;
import roadgraph.MapNode;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.HashMap;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.PriorityQueue;
import java.util.Queue;
import java.util.Set;
import java.util.function.Consumer;

import geography.GeographicPoint;
import util.GraphLoader;
import roadgraph.MapNodeComparator;
/**
 * @author UCSD MOOC development team and YOU
 * 
 * A class which represents a graph of geographic locations
 * Nodes in the graph are intersections between 
 *
 */
public class MapGraph {
	//TODO: Add your member variables here in WEEK 2
	private HashMap<GeographicPoint, MapNode> vertices;
	private int count1 = 0;
	private int count2 = 0;
	
	/** 
	 * Create a new empty MapGraph 
	 */
	public MapGraph()
	{
		// TODO: Implement in this constructor in WEEK 2
	    this.vertices = new HashMap<>();
	}
	
	/**
	 * Get the number of vertices (road intersections) in the graph
	 * @return The number of vertices in the graph.
	 */
	public int getNumVertices()
	{
		//TODO: Implement this method in WEEK 2
		return this.vertices.size();
	}
	
	/**
	 * Return the intersections, which are the vertices in this graph.
	 * @return The vertices in this graph as GeographicPoints
	 */
	public Set<GeographicPoint> getVertices()
	{
		//TODO: Implement this method in WEEK 2
		return this.vertices.keySet();
	}
	
	/**
	 * Get the number of road segments in the graph
	 * @return The number of edges in the graph.
	 */
	public int getNumEdges()
	{
		//TODO: Implement this method in WEEK 2
	    int num = 0;
        for (GeographicPoint temp : this.vertices.keySet()) {
            num += vertices.get(temp).getNeighbors().size();
        }
        
        return num;
	}

	
	
	/** Add a node corresponding to an intersection at a Geographic Point
	 * If the location is already in the graph or null, this method does 
	 * not change the graph.
	 * @param location  The location of the intersection
	 * @return true if a node was added, false if it was not (the node
	 * was already in the graph, or the parameter is null).
	 */
	public boolean addVertex(GeographicPoint location)
	{
		// TODO: Implement this method in WEEK 2
	    if (this.vertices.containsKey(location) || location.equals(null)) return false;
	    
	    this.vertices.put(location, new MapNode(location));
	    return true;
	}
	
	/**
	 * Adds a directed edge to the graph from pt1 to pt2.  
	 * Precondition: Both GeographicPoints have already been added to the graph
	 * @param from The starting point of the edge
	 * @param to The ending point of the edge
	 * @param roadName The name of the road
	 * @param roadType The type of the road
	 * @param length The length of the road, in km
	 * @throws IllegalArgumentException If the points have not already been
	 *   added as nodes to the graph, if any of the arguments is null,
	 *   or if the length is less than 0.
	 */
	public void addEdge(GeographicPoint from, GeographicPoint to, String roadName,
			String roadType, double length) throws IllegalArgumentException {

		//TODO: Implement this method in WEEK 2
	    if (length < 0 || !this.vertices.containsKey(from) || !this.vertices.containsKey(to))
	        throw new IllegalArgumentException("Points are not located on the Map or road length is less than 0");
	    
	    if (from.equals(null) || to.equals(null) || roadName.equals(null) || roadType.equals(null))
	        throw new IllegalArgumentException("Wrong Input.");
	    
	    this.vertices.get(from).addNeighbor(this.vertices.get(to));
	    this.vertices.get(from).addEdge(from, to, roadName, roadType, length);
	   
	}
	
	/*
	 * Get the vertices from the set of geographic points
	 */

	
	/** Find the path from start to goal using breadth first search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @return The list of intersections that form the shortest (unweighted)
	 *   path from start to goal (including both start and goal).
	 */
	public List<GeographicPoint> bfs(GeographicPoint start, GeographicPoint goal) {
		// Dummy variable for calling the search algorithms
        Consumer<GeographicPoint> temp = (x) -> {};
        return bfs(start, goal, temp);
	}
	
	/** Find the path from start to goal using breadth first search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @param nodeSearched A hook for visualization.  See assignment instructions for how to use it.
	 * @return The list of intersections that form the shortest (unweighted)
	 *   path from start to goal (including both start and goal).
	 */
	public List<GeographicPoint> bfs(GeographicPoint start, 
			 					     GeographicPoint goal, Consumer<GeographicPoint> nodeSearched)
	{
		// TODO: Implement this method in WEEK 2
	    Queue<MapNode> toExplore = new LinkedList<>();
	    HashMap<MapNode, MapNode> parent = new HashMap<>();
	    HashSet<MapNode> visited = new HashSet<>();
	    
	    boolean found = false;
	    toExplore.add(this.vertices.get(start));
	    MapNode cur;
	    
	    while (!toExplore.isEmpty()) {
	       
	        cur = toExplore.remove();
	        nodeSearched.accept(cur.getLocation());
	        
	        if (cur.equals(this.vertices.get(goal))) {
	            found = true;
	            break;
	        }
	        
	        List<MapNode> neighbors = cur.getNeighbors();
	        
	        for(MapNode temp : neighbors) {
	            	            
	            if(!visited.contains(temp)) {
	                visited.add(temp);
	                parent.put(temp, cur);
	                toExplore.add(temp);
	            }
	        }
	    }
	    
	    if (!found) {
	        System.out.println("No Path Found");
	        return null;
	    }
	    
	    cur = this.vertices.get(goal);
	    LinkedList<GeographicPoint> path = new LinkedList<>();
	    
	    while (!cur.equals(this.vertices.get(start))) {
	        path.addFirst(cur.getLocation());
	        cur = parent.get(cur);
	    }
	    
	    path.addFirst(start);
		
		// Hook for visualization.  See writeup.
		//nodeSearched.accept(next.getLocation());

		return path;
	}
	

	/** Find the path from start to goal using Dijkstra's algorithm
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> dijkstra(GeographicPoint start, GeographicPoint goal) {
		// Dummy variable for calling the search algorithms
		// You do not need to change this method.
        Consumer<GeographicPoint> temp = (x) -> {};
        return dijkstra(start, goal, temp);
	}
	
	/** Find the path from start to goal using Dijkstra's algorithm
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @param nodeSearched A hook for visualization.  See assignment instructions for how to use it.
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> dijkstra(GeographicPoint start, 
										  GeographicPoint goal, Consumer<GeographicPoint> nodeSearched)
	{
		// TODO: Implement this method in WEEK 3
	    Comparator<MapNode> comparator = new MapNodeComparator();
	    PriorityQueue<MapNode> toExplore = new PriorityQueue<MapNode>(15, comparator);
        HashMap<MapNode, MapNode> parent = new HashMap<>();
        HashSet<MapNode> visited = new HashSet<>();
        
        boolean found = false;
        this.vertices.get(start).setDistance(0);
        toExplore.add(this.vertices.get(start));
        MapNode cur;
        
        while (!toExplore.isEmpty()) {
            cur = toExplore.remove();
            count1++;
            nodeSearched.accept(cur.getLocation());
            
            if (!visited.contains(cur)) {
                
                visited.add(cur);
                
                if (cur.equals(this.vertices.get(goal))) {
                    
                    found = true;
                    break;
                }
                
                List<MapNode> neighbors = cur.getNeighbors();
                
                for(MapNode temp : neighbors) {
                   
                    double weight = cur.getLength(temp) + cur.getDistance();
                    
                    if (temp.getDistance() > weight) {
                        temp.setDistance(weight); 
                        parent.put(temp, cur);
                        toExplore.add(temp);
                    }
                }
            }
        }
        
        if (!found) {
            System.out.println("No Path Found");
            return null;
        }
        
        cur = this.vertices.get(goal);
        LinkedList<GeographicPoint> path = new LinkedList<>();
        
        while (!cur.equals(this.vertices.get(start))) {
            path.addFirst(cur.getLocation());
            cur = parent.get(cur);
        }
        
        path.addFirst(start);
        
        // Hook for visualization.  See writeup.
        //nodeSearched.accept(next.getLocation());
       
        return path;
	
	}

	/** Find the path from start to goal using A-Star search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> aStarSearch(GeographicPoint start, GeographicPoint goal) {
		// Dummy variable for calling the search algorithms
        Consumer<GeographicPoint> temp = (x) -> {};
        return aStarSearch(start, goal, temp);
	}
	
	/** Find the path from start to goal using A-Star search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @param nodeSearched A hook for visualization.  See assignment instructions for how to use it.
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> aStarSearch(GeographicPoint start, 
											 GeographicPoint goal, Consumer<GeographicPoint> nodeSearched)
	{
		// TODO: Implement this method in WEEK 3
	    Comparator<MapNode> comparator = new AStarComparator();
        PriorityQueue<MapNode> toExplore = new PriorityQueue<MapNode>(15, comparator);
        HashMap<MapNode, MapNode> parent = new HashMap<>();
        HashSet<MapNode> visited = new HashSet<>();
        
        boolean found = false;
        this.vertices.get(start).setDistance(0);
        this.vertices.get(start).setHeuristicDistance(goal);
        toExplore.add(this.vertices.get(start));
        MapNode cur;
        
        while (!toExplore.isEmpty()) {
            cur = toExplore.remove();
            nodeSearched.accept(cur.getLocation());
            count2++;
            
            if (!visited.contains(cur)) {
                
                visited.add(cur);
                
                if (cur.equals(this.vertices.get(goal))) {
                    
                    found = true;
                    break;
                }
                
                List<MapNode> neighbors = cur.getNeighbors();
                
                for(MapNode temp : neighbors) {
                   
                    double weight = cur.getLength(temp) + cur.getDistance();
                    
                    if (temp.getDistance() > weight) {
                        temp.setDistance(weight);
                        temp.setHeuristicDistance(goal);
                        parent.put(temp, cur);
                        toExplore.add(temp);
                    }
                }
            }
        }
        
        if (!found) {
            System.out.println("No Path Found");
            return null;
        }
        
        cur = this.vertices.get(goal);
        LinkedList<GeographicPoint> path = new LinkedList<>();
        
        while (!cur.equals(this.vertices.get(start))) {
            path.addFirst(cur.getLocation());
            cur = parent.get(cur);
        }
        
        path.addFirst(start);
        
        // Hook for visualization.  See writeup.
        //nodeSearched.accept(next.getLocation());
        
        return path;
		
	}

	
	
	public static void main(String[] args)
	{
		System.out.print("Making a new map...");
		MapGraph firstMap = new MapGraph();
		System.out.print("DONE. \nLoading the map...");
		GraphLoader.loadRoadMap("data/testdata/simpletest.map", firstMap);
		System.out.println("DONE.");
		
				
		/* Here are some test cases you should try before you attempt 
		 * the Week 3 End of Week Quiz, EVEN IF you score 100% on the 
		 * programming assignment.
		 */
		/*
		MapGraph simpleTestMap = new MapGraph();
		GraphLoader.loadRoadMap("data/testdata/simpletest.map", simpleTestMap);
		
		GeographicPoint testStart = new GeographicPoint(1.0, 1.0);
		GeographicPoint testEnd = new GeographicPoint(8.0, -1.0);
		
		System.out.println("Test 1 using simpletest: Dijkstra should be 9 and AStar should be 5");
		List<GeographicPoint> testroute2 = simpleTestMap.aStarSearch(testStart,testEnd);
		List<GeographicPoint> testroute = simpleTestMap.dijkstra(testStart,testEnd);
		
		System.out.println("Number of Nodes Dijkstra: " + simpleTestMap.count1);
		System.out.println("Number of Nodes A*: " + simpleTestMap.count2);
		
		
		MapGraph testMap = new MapGraph();
		GraphLoader.loadRoadMap("data/maps/utc.map", testMap);
		
		// A very simple test using real data
		testStart = new GeographicPoint(32.869423, -117.220917);
		testEnd = new GeographicPoint(32.869255, -117.216927);
		System.out.println("Test 2 using utc: Dijkstra should be 13 and AStar should be 5");
		testroute = testMap.dijkstra(testStart,testEnd);
		testroute2 = testMap.aStarSearch(testStart,testEnd);
		System.out.println("Number of Nodes Dijkstra: " + testMap.count1);
        System.out.println("Number of Nodes A*: " + testMap.count2);
		
		
		// A slightly more complex test using real data
		testStart = new GeographicPoint(32.8674388, -117.2190213);
		testEnd = new GeographicPoint(32.8697828, -117.2244506);
		System.out.println("Test 3 using utc: Dijkstra should be 37 and AStar should be 10");
		testroute = testMap.dijkstra(testStart,testEnd);
		testroute2 = testMap.aStarSearch(testStart,testEnd);
		System.out.println("Number of Nodes Dijkstra: " + simpleTestMap.count1);
        System.out.println("Number of Nodes A*: " + simpleTestMap.count2);
        */
		
        MapGraph theMap = new MapGraph();
        System.out.print("DONE. \nLoading the map...");
        GraphLoader.loadRoadMap("data/maps/utc.map", theMap);
        System.out.println("DONE.");
        
        GeographicPoint end = new GeographicPoint(32.8660691, -117.217393);
        GeographicPoint start = new GeographicPoint(32.8648772, -117.2254046);
        
        
        List<GeographicPoint> route = theMap.dijkstra(start,end);
        List<GeographicPoint> route2 = theMap.aStarSearch(start,end);
        
        
        System.out.println("Number of Nodes Dijkstra: " + theMap.count1);
        System.out.println("Number of Nodes A*: " + theMap.count2);
       
		/* Use this code in Week 3 End of Week Quiz */
		/*MapGraph theMap = new MapGraph();
		
		
		System.out.print("DONE. \nLoading the map...");
		GraphLoader.loadRoadMap("data/maps/utc.map", theMap);
		System.out.println("DONE.");

		GeographicPoint start = new GeographicPoint(32.8648772, -117.2254046);
		GeographicPoint end = new GeographicPoint(32.8660691, -117.217393);
		
		
		List<GeographicPoint> route = theMap.dijkstra(start,end);
		List<GeographicPoint> route2 = theMap.aStarSearch(start,end);

		*/
		
	}
	
}
