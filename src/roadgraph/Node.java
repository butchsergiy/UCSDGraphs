package roadgraph;

import geography.GeographicPoint;

import java.util.HashMap;
import java.util.HashSet;

/**
 * Created by BSV on 29.09.2016.
 */
public class Node {

    private final GeographicPoint geographicPoint;

    private HashSet<Edge> neighbours = new HashSet<>();



    public Node(GeographicPoint geographicPoint) {
        this.geographicPoint = geographicPoint;
    }

    public GeographicPoint getGeographicPoint() {
        return new GeographicPoint(geographicPoint.getX(),geographicPoint.getY());
    }


    public HashSet<Edge> getNeighbours() {
        return new HashSet<>(neighbours);
    }

    public boolean addNeighbours(Edge neighbours) {
        return this.neighbours.add(neighbours);
    }


    @Override
    public boolean equals(Object o) {
        if (this == o) return true;
        if (o == null || getClass() != o.getClass()) return false;

        Node node = (Node) o;

        return geographicPoint.equals(node.geographicPoint);
    }

    @Override
    public int hashCode() {
        return geographicPoint.hashCode();
    }

}
