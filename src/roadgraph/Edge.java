package roadgraph;

import geography.GeographicPoint;

/**
 * Class represents edge between two GeographicPoints (lat, lon pairs) that are already in the graph.
 * The edges represent streets segments, and hence they connect intersections.
 * @param roadName is the name of the road (e.g. "Main street")
 * @param roadType The type of the road(e.g. "residential").
 * @param length is the length of this road segment, in km.
 */
public class Edge {

    private final GeographicPoint from;
    private final GeographicPoint to;
    private final String roadName;
    private final String roadType;      // the roadType is the kind of road (e.g. "residential").
    private final double length;        // the length of this road segment, in km.

    public Edge(GeographicPoint from, GeographicPoint to, String roadName, String roadType, double length) {
        this.from = from;
        this.to = to;
        this.roadName = roadName;
        this.roadType = roadType;
        this.length = length;
    }

    @Override
    public boolean equals(Object o) {
        if (this == o) return true;
        if (o == null || getClass() != o.getClass()) return false;

        Edge edge = (Edge) o;

        if (!from.equals(edge.from)) return false;
        if (!to.equals(edge.to)) return false;
        return roadName.equals(edge.roadName);

    }

    @Override
    public int hashCode() {
        int result = from.hashCode();
        result = 31 * result + to.hashCode();
        result = 31 * result + roadName.hashCode();
        return result;
    }

    @Override
    public String toString() {
        return "Edge{" +
                "from=" + from +
                ", to=" + to +
                ", roadName='" + roadName + '\'' +
                ", roadType='" + roadType + '\'' +
                ", length=" + length +
                '}';
    }
}
