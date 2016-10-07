package roadgraph;

import geography.GeographicPoint;

/**
 *  Class to implement PriorityQueue for Dijkstra algorithm
 */
class NodeForPriorityQueue {

    private GeographicPoint nodeGeoPoint;
    private double distanceToStartNode;

    public NodeForPriorityQueue(GeographicPoint nodeGeoPoint, double distanceToStartNode) {
        this.nodeGeoPoint = nodeGeoPoint;
        this.distanceToStartNode = distanceToStartNode;
    }

    public GeographicPoint getNodeGeoPoint() {
        return nodeGeoPoint;
    }

    public void setNodeGeoPoint(GeographicPoint nodeGeoPoint) {
        this.nodeGeoPoint = nodeGeoPoint;
    }

    public double getDistanceToStartNode() {
        return distanceToStartNode;
    }

    public void setDistanceToStartNode(double distanceToStartNode) {
        this.distanceToStartNode = distanceToStartNode;
    }
}
