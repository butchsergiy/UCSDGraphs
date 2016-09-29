package roadgraph;

import geography.GeographicPoint;

import java.util.HashMap;

/**
 * Created by BSV on 29.09.2016.
 */
public class Node {

    private final GeographicPoint geographicPoint;
    private HashMap <GeographicPoint, Float> neighbours = new HashMap<>();

    public Node(GeographicPoint geographicPoint, HashMap<GeographicPoint, Float> neighbours) {
        this.geographicPoint = geographicPoint;
        this.neighbours = neighbours;
    }

    public GeographicPoint getGeographicPoint() {
        return geographicPoint;
    }

    public HashMap<GeographicPoint, Float> getNeighbours() {
        return neighbours;
    }

    public void addNeighbour(Node neighbour) {
        this.neighbours.put(neighbour.getGeographicPoint(), 0f);
    }
    public void addNeighbour(Node neighbour, Float distanceToNeighbour) {
        this.neighbours.put(neighbour.getGeographicPoint(), distanceToNeighbour);
    }
}
