package roadgraph;

import java.util.Comparator;

public class AStarComparator implements Comparator<MapNode> {

    public int compare(MapNode location1, MapNode location2) {
        
        double distance1 = location1.getDistance() + location1.getHeuristicDistance();
        double distance2 = location2.getDistance() + location2.getHeuristicDistance();
        if (distance1 < distance2)
        {
            return -1;
        }
        if (distance1 > distance2)
        {
            return 1;
        }
        return 0;
    }
}
