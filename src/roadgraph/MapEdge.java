package roadgraph;

import geography.GeographicPoint;

public class MapEdge {
    private GeographicPoint start;
    private GeographicPoint end;
    private String roadName;
    private String roadType;
    private double length;
    
    
    public MapEdge(GeographicPoint newStart, GeographicPoint newEnd, String newRoadName, String newRoadType, double newLength){
        this.start = newStart;
        this.end = newEnd;
        this.roadName = newRoadName;
        this.roadType = newRoadType;
        this.length = newLength;   
    }
    
    public GeographicPoint getStart() {
        return this.start;
    }
    
    public GeographicPoint getEnd() {
        return this.end;
    }
    
    public String getRoadName() {
        return this.roadName;
    }
    
    public String getRoadType() {
        return this.roadType;
    }
    
    public double getlength() {
        return this.length;
    }
    
    public void setRoadName(String name) {
        this.roadName = name;
    }
    
    public void setRoadType(String type) {
        this.roadType = type;
    }
    
    public void setLength(double length) {
        this.length = length;
    }
}
