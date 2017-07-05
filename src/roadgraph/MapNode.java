package roadgraph;
import geography.GeographicPoint;
import roadgraph.MapEdge;

import java.util.Comparator;
import java.util.HashMap;
import java.util.LinkedList;
import java.util.List;

public class MapNode {

    private GeographicPoint location;
    private List<MapNode> neighbors;
    private List<MapEdge> edges;
    private double distance;
    private double heuristicDistance;

    public MapNode (GeographicPoint newLocation) {
        this.location = newLocation;
        this.neighbors = new LinkedList<>();
        this.edges = new LinkedList<>();
        this.distance = Double.POSITIVE_INFINITY;
        this.heuristicDistance = Double.POSITIVE_INFINITY;
    }
    
    public void setHeuristicDistance(GeographicPoint destination) {
        this.heuristicDistance = this.getLocation().distance(destination);
    }
    
    public double getHeuristicDistance() {
        return this.heuristicDistance;
    }
          
    public GeographicPoint getLocation() {
        return this.location;
    }
    
    public void setLocation(GeographicPoint location) {
        this.location = location;
    }
    
    public List<MapNode> getNeighbors() {
        return this.neighbors;
    }
    
    public void addNeighbor(MapNode node){
       this.neighbors.add(node);        
    }
    
    public void addEdge(GeographicPoint from, GeographicPoint to, String roadName, String roadType, double length) {
        MapEdge road = new MapEdge(from, to, roadName, roadType, length);
        this.edges.add(road);
    }
    
    public List<MapEdge> getEdges() {
        return this.edges;
    }
    
    public double getLength(MapNode location) {
        
        double length = 0;
        for (MapEdge temp : this.getEdges()) {
            if (temp.getEnd().equals(location.getLocation())) {
              length = temp.getlength();  
              break;
            }
        }
        return length;
    }
    
    public double getDistance() {
        return this.distance;
    }
    
    public void setDistance(double newDistance) {
        this.distance = newDistance;
    }
    
    public String toString() {
        return "Location: " + this.getLocation();
    }
}
