package roadgraph;

import geography.GeographicPoint;

/**
 *  Class to implement PriorityQueue for Dijkstra algorithm
 */
class NodeForPriorityQueue {

    private GeographicPoint nodeGeoPoint;
    private double distanceToStartNode;
    private double distanceToEndNode;

    public NodeForPriorityQueue(GeographicPoint nodeGeoPoint, double distanceToStartNode) {
        this.nodeGeoPoint = nodeGeoPoint;
        this.distanceToStartNode = distanceToStartNode;
    }

    public NodeForPriorityQueue(GeographicPoint nodeGeoPoint, double distanceToStartNode, double distanceToEndNode) {
        this.nodeGeoPoint = nodeGeoPoint;
        this.distanceToStartNode = distanceToStartNode;
        this.distanceToEndNode = distanceToEndNode;
    }

    public double getDistanceToEndNode() {
        return distanceToEndNode;
    }


    public void setDistanceToEndNode(double distanceToEndNode) {
        this.distanceToEndNode = distanceToEndNode;
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
