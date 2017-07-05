package roadgraph;

import java.util.Comparator;

public class MapNodeComparator implements Comparator<MapNode> {

    public int compare(MapNode location1, MapNode location2) {
        if (location1.getDistance() < location2.getDistance())
        {
            return -1;
        }
        if (location1.getDistance() > location2.getDistance())
        {
            return 1;
        }
        return 0;
    }
}
