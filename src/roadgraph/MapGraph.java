/**
 * @author UCSD MOOC development team and YOU
 * 
 * A class which reprsents a graph of geographic locations
 * Nodes in the graph are intersections between 
 *
 */
package roadgraph;


import java.util.*;
import java.util.function.BiConsumer;
import java.util.function.Consumer;

import geography.GeographicPoint;
import util.GraphLoader;

/**
 * @author UCSD MOOC development team and ButchSergiy
 * 
 * A class which represents a graph of geographic locations
 * Nodes in the graph are intersections between 
 *
 */
public class MapGraph {
	//TODO: Add your member variables here in WEEK 3

	//	private List<Node> verticesArrayList = new ArrayList<>();
//	private List<Node> verticesLinkedList = new LinkedList<>();
	private HashSet<Node> verticesHashSet = new HashSet<>();
	//	private HashSet<Edge> edgesHashSet = new HashSet<>();
	private int numVertices = 0;
	private int numEdges = 0;

	/**
	 * Create a new empty MapGraph
	 */
	public MapGraph() {
		// TODO: Implement in this constructor in WEEK 3
	}

	/**
	 * Get the number of vertices (road intersections) in the graph
	 *
	 * @return The number of vertices in the graph.
	 */
	public int getNumVertices() {
		return this.numVertices;
	}

	/**
	 * Get the number of road segments in the graph
	 *
	 * @return The number of edges in the graph.
	 */
	public int getNumEdges() {
		return this.numEdges;
	}


	/**
	 * Return the intersections, which are the vertices in this graph.
	 *
	 * @return The vertices in this graph as GeographicPoints
	 */
	public Set<GeographicPoint> getVertices() {
		Set<GeographicPoint> vertices = new HashSet<>();
		verticesHashSet.forEach(v -> vertices.add(v.getGeographicPoint()));
		return vertices;
	}


	/**
	 * Add a node corresponding to an intersection at a Geographic Point
	 * If the location is already in the graph or null, this method does
	 * not change the graph.
	 *
	 * @param location The location of the intersection
	 * @return true if a node was added, false if it was not (the node
	 * was already in the graph, or the parameter is null).
	 */
	public boolean addVertex(GeographicPoint location) {
		if (location == null) return false;
		if (verticesHashSet.add(new Node(location))) {
			this.numVertices++;
			return true;
		}
		return false;
	}

	/**
	 * Adds a directed edge to the graph from pt1 to pt2.
	 * Precondition: Both GeographicPoints have already been added to the graph
	 *
	 * @param from     The starting point of the edge
	 * @param to       The ending point of the edge
	 * @param roadName The name of the road
	 * @param roadType The type of the road
	 * @param length   The length of the road, in km
	 * @throws IllegalArgumentException If the points have not already been
	 *                                  added as nodes to the graph, if any of the arguments is null,
	 *                                  or if the length is less than 0.
	 */
	public void addEdge(GeographicPoint from, GeographicPoint to, String roadName,
						String roadType, double length) throws IllegalArgumentException {

		if (!verticesHashSet.contains(new Node(from))
				|| !verticesHashSet.contains(new Node(to))
				|| from == null
				|| to == null
				|| roadName == null
				|| roadType == null
				|| length < 0
				)
			throw new IllegalArgumentException("Either the two points are not in the graph already, any of the arguments is null, or if length is less than 0.");

		for (Node node : verticesHashSet) {
			if (node.getGeographicPoint().equals(from)) {
				node.addNeighbours(new Edge(from, to, roadName, roadType, length));
			}
		}
		this.numEdges++;
	}


	/**
	 * Find the path from start to goal using breadth first search
	 *
	 * @param start The starting location
	 * @param goal  The goal location
	 * @return The list of intersections that form the shortest (unweighted)
	 * path from start to goal (including both start and goal).
	 */
	public List<GeographicPoint> bfs(GeographicPoint start, GeographicPoint goal) {
		// Dummy variable for calling the search algorithms
		Consumer<GeographicPoint> temp = (x) -> {
		};
		return bfs(start, goal, temp);
	}

	/**
	 * Find the path from start to goal using breadth first search
	 *
	 * @param start        The starting location
	 * @param goal         The goal location
	 * @param nodeSearched A hook for visualization.  See assignment instructions for how to use it.
	 * @return The list of intersections that form the shortest (unweighted)
	 * path from start to goal (including both start and goal).
	 */
	public List<GeographicPoint> bfs(GeographicPoint start,
									 GeographicPoint goal, Consumer<GeographicPoint> nodeSearched) {
		// Hook for visualization.  See writeup.
		//nodeSearched.accept(next.getLocation());

		Queue<Node> nodeQueue = new LinkedList<>();
		HashSet<Node> visitedSet = new HashSet<>();
		HashMap<Node, Node> pathMap = new HashMap(); // parents map

		Node startNode = getNode(start);
		Node endNode = getNode(goal);
		Node currentNode;
		nodeQueue.add(startNode);
		visitedSet.add(startNode);

		while (!nodeQueue.isEmpty()) {
			currentNode = nodeQueue.poll();
			if (currentNode.equals(endNode)) break;

			// put not visited current node neighbours to queue, visitedSet and pathMap(parents map)
			for (Node nodeNext : getNeighboursCollection(currentNode)) {
				if (!visitedSet.contains(nodeNext)) {
					nodeQueue.add(nodeNext);
					visitedSet.add(nodeNext);
					pathMap.put(nodeNext, currentNode);
				}
			}
		}
		return convertMapToList(pathMap, startNode,endNode);
	}

	private Node getNode(GeographicPoint start) {
		for (Node node : verticesHashSet) {
			if (node.getGeographicPoint().equals(start))
				return node;
		}
		return null;
	}

	public HashSet<Node> getNeighboursCollection(Node node) {
		HashSet<Node> neighboursHashSet = new HashSet<>();
		for (Edge neighbour : node.getNeighbours()) {
			neighboursHashSet.add(getNode(neighbour.getTo()));
		}
	return neighboursHashSet;
	}


	private List<GeographicPoint> convertMapToList(HashMap<Node, Node> path, Node startNode, Node endNode) {

		LinkedList<GeographicPoint> pathList = new LinkedList();
		Node currentNode = endNode;
		Node parent;

		pathList.add(endNode.getGeographicPoint());

		// pathMap is empty because start point and end point are same and there aren't any parents in map.
		if(path.isEmpty()) return pathList;

		while(path.containsKey(currentNode)){
			parent = path.get(currentNode);
			pathList.addFirst(parent.getGeographicPoint());
			currentNode = parent;
		}

		return pathList;
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
		// TODO: Implement this method in WEEK 4

		// Hook for visualization.  See writeup.
		//nodeSearched.accept(next.getLocation());
		
		return null;
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
		// TODO: Implement this method in WEEK 4
		
		// Hook for visualization.  See writeup.
		//nodeSearched.accept(next.getLocation());
		
		return null;
	}



	private void printGraph() {
		System.out.println("Graph Data: \t" + "Number of vertices:\t" +this.numVertices + "\t Number of edges: \t" + this.numEdges);
		int i = 1;

		for (Node node : verticesHashSet) {

			System.out.print("\nNode #" + (i++) + " geo point: " + node.getGeographicPoint()
				+ "\nNode neighbours:\n");
			node.getNeighbours().forEach((v) -> System.out.println(v));
		}
	}

	public static void main(String[] args)
	{
		System.out.print("Making a new map...");
		MapGraph firstMap = new MapGraph();
		System.out.print("DONE. \nLoading the map...");
		GraphLoader.loadRoadMap("data/testdata/simpletest.map", firstMap);
		System.out.println("DONE.");

		// You can use this method for testing.


		/* Here are some test cases you should try before you attempt
		 * the Week 3 End of Week Quiz, EVEN IF you score 100% on the
		 * programming assignment.
		 */

		MapGraph simpleTestMap = new MapGraph();
		GraphLoader.loadRoadMap("data/testdata/simpletest.map", simpleTestMap);

		GeographicPoint testStart = new GeographicPoint(1.0, 1.0);
		GeographicPoint testEnd = new GeographicPoint(8.0, -1.0);

		List<GeographicPoint> testroute = simpleTestMap.bfs(testStart,testEnd);
		simpleTestMap.printGraph();

		System.out.println(testroute);


//		System.out.println("Test 1 using simpletest: Dijkstra should be 9 and AStar should be 5");
//		List<GeographicPoint> testroute1 = simpleTestMap.dijkstra(testStart,testEnd);
//		List<GeographicPoint> testroute2 = simpleTestMap.aStarSearch(testStart,testEnd);


//		MapGraph testMap = new MapGraph();
//		GraphLoader.loadRoadMap("data/maps/utc.map", testMap);

//		// A very simple test using real data
//		testStart = new GeographicPoint(32.869423, -117.220917);
//		testEnd = new GeographicPoint(32.869255, -117.216927);
//		System.out.println("Test 2 using utc: Dijkstra should be 13 and AStar should be 5");
//		testroute = testMap.dijkstra(testStart,testEnd);
//		testroute2 = testMap.aStarSearch(testStart,testEnd);
//
//
//		// A slightly more complex test using real data
//		testStart = new GeographicPoint(32.8674388, -117.2190213);
//		testEnd = new GeographicPoint(32.8697828, -117.2244506);
//		System.out.println("Test 3 using utc: Dijkstra should be 37 and AStar should be 10");
//		testroute = testMap.dijkstra(testStart,testEnd);
//		testroute2 = testMap.aStarSearch(testStart,testEnd);



//		/* Use this code in Week 3 End of Week Quiz */
//		MapGraph theMap = new MapGraph();
//		System.out.print("DONE. \nLoading the map...");
//		GraphLoader.loadRoadMap("data/maps/utc.map", theMap);
//		System.out.println("DONE.");
//
//		GeographicPoint start = new GeographicPoint(32.8648772, -117.2254046);
//		GeographicPoint end = new GeographicPoint(32.8660691, -117.217393);
//
//
//		List<GeographicPoint> route = theMap.dijkstra(start,end);
//		List<GeographicPoint> route2 = theMap.aStarSearch(start,end);

	}

}
