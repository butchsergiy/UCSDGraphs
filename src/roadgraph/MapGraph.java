/**
 * @author UCSD MOOC development team and BSV
 * 
 * A class which reprsents a graph of geographic locations
 * Nodes in the graph are intersections between 
 *
 */
package roadgraph;

import geography.GeographicPoint;
import util.GraphLoader;

import java.util.*;
import java.util.function.Consumer;

import org.perf4j.log4j.Log4JStopWatch;

/**
 * @author UCSD MOOC development team and ButchSergiy
 * 
 * A class which represents a graph of geographic locations
 * Nodes in the graph are intersections between 
 *
 */
public class MapGraph {

	private HashMap<GeographicPoint, LinkedList<Edge>> nodesHashMap = new HashMap<>();

	private int numVertices = 0;
	private int numEdges = 0;

	/**
	 * Simple stopWatch to measure operation last time.
	 */
	Log4JStopWatch sw1 = new Log4JStopWatch("MyLogger");


	/**
	 * Create a new empty MapGraph
	 */
	public MapGraph() {
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
		if (!nodesHashMap.containsKey(location)) {
			this.numVertices++;
			nodesHashMap.put(location, new LinkedList<>());
			return true;
		}
		return false;
	}


	/**
	 * Return the intersections, which are the vertices in this graph.
	 *
	 * @return The vertices in this graph as GeographicPoints
	 */
	public Set<GeographicPoint> getVertices() {
		return nodesHashMap.keySet();
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

		if (from == null || to == null
				|| roadName == null
				|| roadType == null
				|| length < 0
				|| !nodesHashMap.containsKey(from)
				|| !nodesHashMap.containsKey(to)
				)
			throw new IllegalArgumentException("Either the two points are not in the graph already, any of the arguments is null, or if length is less than 0.");

		nodesHashMap.get(from).add(new Edge(from, to, roadName, roadType, length));
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

		sw1.start();

		Queue<GeographicPoint> nodeQueue = new LinkedList<>();
		HashSet<GeographicPoint> visitedSet = new HashSet<>();
		HashMap<GeographicPoint, GeographicPoint> parentsMap = new HashMap();

		GeographicPoint currentNode;
		nodeQueue.add(start);
		visitedSet.add(start);
		Double distance = 0d;


		while (!nodeQueue.isEmpty()) {
			currentNode = nodeQueue.poll();
			if (currentNode.equals(goal)) break;

			// iterate over list of edges of the Node
			// and for each Edge check its second point
			for (Edge nextEdge : nodesHashMap.get(currentNode)) {
				GeographicPoint neighbourNode = nextEdge.getTo();

				// Hook for visualization.  See writeup.
				nodeSearched.accept(neighbourNode);

				if (!visitedSet.contains(neighbourNode)) {
					nodeQueue.add(neighbourNode);
					visitedSet.add(neighbourNode);
					parentsMap.put(neighbourNode, currentNode);
				}
			}
		}

		List<GeographicPoint> geographicPoints = convertMapToList(parentsMap, goal);

		sw1.stop();
		System.out.print("\n-- SEARCH method BFS. Search time in ms: " + sw1.getElapsedTime());

		// Print more information just for tests
		if(geographicPoints == null) System.out.println("SEARCH method BFS didnt find any road.");
		else printMethodSearchStatistic("SEARCH method BFS.", start, goal, geographicPoints);

		System.out.println(" -- Visited Set size: " + visitedSet.size() + "\t VisitedSet: " + visitedSet);

		return geographicPoints;
	}

	private void printMethodSearchStatistic(String text, GeographicPoint start, GeographicPoint goal, List<GeographicPoint> geographicPoints) {
		System.out.println("\n-- " + text);
		System.out.println("-- FROM: " + start + "\t TO: " + goal );
		System.out.println("-- Node quantity: " + (geographicPoints==null ? null : geographicPoints.size()) + "\t Length: " + getDistance(geographicPoints));
		System.out.println(geographicPoints);
	}

	private double getDistance(List<GeographicPoint> geographicPoints) {
		// method to search path length
		double distance = 0d;
		GeographicPoint parent = geographicPoints.get(0);

		for (GeographicPoint node : geographicPoints) {

			for (Edge edge : nodesHashMap.get(parent)) {
				if (edge.getTo().equals(node)) {
					distance += edge.getLength();
				}
			}
			parent = node;
		}
		return distance;
	}


	private List<GeographicPoint> convertMapToList(HashMap<GeographicPoint, GeographicPoint> path, GeographicPoint endNode) {

		LinkedList<GeographicPoint> pathList = new LinkedList<>();
		GeographicPoint currentNode = endNode;
		GeographicPoint parent;

		pathList.add(endNode);


		// pathMap is empty because start point and end point are same and there aren't any parents in map.
		if(path.isEmpty()) return null;

		while(path.containsKey(currentNode)){
			parent = path.get(currentNode);
			pathList.addFirst(parent);
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
										  GeographicPoint goal, Consumer<GeographicPoint> nodeSearched) {
		sw1.start();

		Comparator<NodeForPriorityQueue> nodePQueueComparator = new Comparator<NodeForPriorityQueue>() {
			@Override
			public int compare(NodeForPriorityQueue o1, NodeForPriorityQueue o2) {
				return ((o1.getDistanceToStartNode() - o2.getDistanceToStartNode())
						<0 ? -1 : 1);
			}
		};

		PriorityQueue<NodeForPriorityQueue> nodePriorityQueue = new PriorityQueue<>(nodePQueueComparator);

		// Set is 5 times faster than LinkedList here,
		// but LinkedList gives you order in with Nodes were visited
//		LinkedList<GeographicPoint> visited = new LinkedList<>();
		Set<GeographicPoint> visited = new HashSet<>();

		HashMap<NodeForPriorityQueue, List<GeographicPoint>> parentsMap = new HashMap();
		HashMap<NodeForPriorityQueue, NodeForPriorityQueue> parentsMap2 = new HashMap();
		double distanceToCurrentNode = 0d;

		GeographicPoint currentNodeGeoPoint;
		NodeForPriorityQueue currentNodeForPQ;
		nodePriorityQueue.add(new NodeForPriorityQueue(start,0d));
		visited.add(start);

		while (!nodePriorityQueue.isEmpty()) {
			distanceToCurrentNode = nodePriorityQueue.peek().getDistanceToStartNode();
			currentNodeForPQ = nodePriorityQueue.poll();
			currentNodeGeoPoint = currentNodeForPQ.getNodeGeoPoint();
			nodeSearched.accept(currentNodeGeoPoint);

			if(!visited.contains(currentNodeGeoPoint)) {
				visited.add(currentNodeGeoPoint);
			}

			if (currentNodeGeoPoint.equals(goal)) break;


			// iterate over list of edges of the Node
			// and for each Edge check its second point
			NodeForPriorityQueue neighbourNodeForPQ;
			for (Edge nextEdge : nodesHashMap.get(currentNodeGeoPoint)) {
				if (!visited.contains(nextEdge.getTo())) {

					neighbourNodeForPQ = new NodeForPriorityQueue(nextEdge.getTo(), distanceToCurrentNode + nextEdge.getLength());

					if (parentsMap2.get(neighbourNodeForPQ) == null
							|| parentsMap2.get(neighbourNodeForPQ).getDistanceToStartNode() > neighbourNodeForPQ.getDistanceToStartNode()) {
						nodePriorityQueue.add(neighbourNodeForPQ);

						List parentsList = new LinkedList();
						if (parentsMap.get(currentNodeForPQ) != null) parentsList.addAll(parentsMap.get(currentNodeForPQ));
						parentsList.add(currentNodeGeoPoint);

						parentsMap.put(neighbourNodeForPQ, parentsList);
						parentsMap2.put(neighbourNodeForPQ, neighbourNodeForPQ);
					}
				}
			}
		}

		List<GeographicPoint> geographicPointsList = parentsMap.get(new NodeForPriorityQueue(goal, 0d));
		geographicPointsList.add(goal);

		sw1.stop();
		System.out.print("\n-- SEARCH method DIJKSTRA. Run time in ms: " + sw1.getElapsedTime());

		// Print more information just for tests
		if(geographicPointsList == null) System.out.println("SEARCH method DIJKSTRA didn't find any road.");
		else printMethodSearchStatistic("SEARCH method DIJKSTRA.", start, goal, geographicPointsList);
		System.out.println(" -- Visited Set size: " + visited.size() + "\t VisitedSet: " + visited);

		return geographicPointsList;
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
											 GeographicPoint goal, Consumer<GeographicPoint> nodeSearched) {

		sw1.start();

		Comparator<NodeForPriorityQueue> nodePQueueComparator = new Comparator<NodeForPriorityQueue>() {
			@Override
			public int compare(NodeForPriorityQueue o1, NodeForPriorityQueue o2) {
				return (((o1.getDistanceToStartNode() + o1.getDistanceToEndNode())
						- (o2.getDistanceToStartNode() + o2.getDistanceToEndNode()))
						< 0 ? -1 : 1);
			}
		};


		PriorityQueue<NodeForPriorityQueue> nodePriorityQueue = new PriorityQueue<>(nodePQueueComparator);

		// Set is 5 times faster than LinkedList here,
		// but LinkedList gives you order in with Nodes were visited
//		LinkedList<GeographicPoint> visited = new LinkedList<>();
		Set<GeographicPoint> visited = new HashSet<>();

		HashMap<NodeForPriorityQueue, List<GeographicPoint>> parentsMap = new HashMap();
		HashMap<NodeForPriorityQueue, NodeForPriorityQueue> parentsMap2 = new HashMap();
		double distanceToCurrentNode = 0d;

		GeographicPoint currentNodeGeoPoint;
		NodeForPriorityQueue currentNodeForPQ;
		nodePriorityQueue.add(new NodeForPriorityQueue(start, 0d));
		visited.add(start);

		while (!nodePriorityQueue.isEmpty()) {
			distanceToCurrentNode = nodePriorityQueue.peek().getDistanceToStartNode();
			currentNodeForPQ = nodePriorityQueue.poll();
			currentNodeGeoPoint = currentNodeForPQ.getNodeGeoPoint();
			nodeSearched.accept(currentNodeGeoPoint);

			if (!visited.contains(currentNodeGeoPoint)) {
				visited.add(currentNodeGeoPoint);
			}

			if (currentNodeGeoPoint.equals(goal)) break;

			// iterate over list of edges of the Node
			// and for each Edge check its second point
			NodeForPriorityQueue neighbourNodeForPQ;
			for (Edge nextEdge : nodesHashMap.get(currentNodeGeoPoint)) {
				if (!visited.contains(nextEdge.getTo())) {

					neighbourNodeForPQ = new NodeForPriorityQueue(nextEdge.getTo(), distanceToCurrentNode + nextEdge.getLength(), nextEdge.getTo().distance(goal));

					//noinspection Duplicates (its for IdeaIDE)
					if (parentsMap2.get(neighbourNodeForPQ) == null
							|| parentsMap2.get(neighbourNodeForPQ).getDistanceToStartNode() > neighbourNodeForPQ.getDistanceToStartNode()) {
						nodePriorityQueue.add(neighbourNodeForPQ);

						List parentsList = new LinkedList();
						if (parentsMap.get(currentNodeForPQ) != null)
							parentsList.addAll(parentsMap.get(currentNodeForPQ));
						parentsList.add(currentNodeGeoPoint);

						parentsMap.put(neighbourNodeForPQ, parentsList);
						parentsMap2.put(neighbourNodeForPQ, neighbourNodeForPQ);
					}
				}
			}
		}

		List<GeographicPoint> geographicPointsList = parentsMap.get(new NodeForPriorityQueue(goal, 0d));
		geographicPointsList.add(goal);

		sw1.stop();
		System.out.print("\n-- SEARCH method ASTAR. Run time in ms: " + sw1.getElapsedTime());

		// Print more information just for tests
		if (geographicPointsList == null) System.out.println("SEARCH method ASTAR didn't find any road.");
		else printMethodSearchStatistic("SEARCH method ASTAR.", start, goal, geographicPointsList);
		System.out.println(" -- Visited Set size: " + visited.size() + "\t VisitedSet: " + visited);

		return geographicPointsList;
	}


	/**
	 *  for testing.
	 *  Method prints each Node and its
	 */
	private void printGraph() {
		System.out.println("Graph Data: \t" + "Number of vertices:\t" +this.numVertices + "\t Number of edges: \t" + this.numEdges);
		int i = 1;

		for (GeographicPoint node : nodesHashMap.keySet()) {

			System.out.print("\nNode #" + (i++) + " geo point: " + node
				+ "\nNode neighbours:\n");
			nodesHashMap.get(node).forEach(System.out::println);
		}
	}


	public static void main(String[] args)
	{

		MapGraph lvivMap = new MapGraph();
		GraphLoader.loadRoadMap("data/maps/lviv.map", lvivMap);
		System.out.println("Lviv map. Number of Nodes: " + lvivMap.numVertices);

		// SEARCH method BFS = 0ms, DIJKSTRA=2ms, ASTAR=0.
//		GeographicPoint Start = new GeographicPoint(49.8119558, 23.9845783);
//		GeographicPoint End = new GeographicPoint(49.8139516, 23.9761248);

		// SEARCH method BFS = 0ms, DIJKSTRA=3ms, ASTAR=0.
// 		GeographicPoint Start = new GeographicPoint(49.8139516, 23.9761248);
//		GeographicPoint End = new GeographicPoint(49.8206981, 23.9689158);

		// SEARCH method BFS = 0ms, DIJKSTRA=1ms, ASTAR=0.
// 		GeographicPoint Start = new GeographicPoint(49.8113121, 23.991146);
//		GeographicPoint End = new GeographicPoint(49.8106745, 23.9974333);

		// SEARCH method BFS = 4ms, DIJKSTRA=140ms, ASTAR=4.
		GeographicPoint Start = new GeographicPoint( 49.8061091,  24.0189236);
		GeographicPoint End = new GeographicPoint(49.8716444, 24.0466452);

//		simpleTestMap.printGraph();

		List<GeographicPoint> lvivRoute1 = lvivMap.bfs(Start,End);
		List<GeographicPoint> lvivRoute2 = lvivMap.dijkstra(Start,End);
		List<GeographicPoint> lvivRoute3 = lvivMap.aStarSearch(Start,End);
//
//		System.out.println("\n* FROM: [" + testStart + "] TO: [" + testEnd + "]\n" +  lvivRoute1);
//		System.out.println("EDGES: " + lvivRoute1.size());
//
//		System.out.println("\n* FROM: [" + testStart + "] TO: [" + testEnd + "]\n" +  lvivRoute2);
//		System.out.println("EDGES: " + lvivRoute2.size());

		System.out.println("/////////////////////////////////////////////////////////////////////////////////////////");


//
//		MapGraph simpleTestMap = new MapGraph();
//		GraphLoader.loadRoadMap("data/testdata/simpletest.map", simpleTestMap);
//
//		testStart = new GeographicPoint(1.0, 1.0);
//		testEnd = new GeographicPoint(8.0, -1.0);
//
//		List<GeographicPoint> testrouteDFS = simpleTestMap.bfs(testStart,testEnd);
//		List<GeographicPoint> testrouteDIJ = simpleTestMap.dijkstra(testStart,testEnd);
//		List<GeographicPoint> testrouteAStar = simpleTestMap.aStarSearch(testStart,testEnd);


		System.out.println("/////////////////////////////////////////////////////////////////////////////////////////");

//		MapGraph testMap = new MapGraph();
//		GraphLoader.loadRoadMap("data/maps/utc.map", testMap);
//
//		// A very simple test using real data
//		testStart = new GeographicPoint(32.869423, -117.220917);
//		testEnd = new GeographicPoint(32.869255, -117.216927);
//
//		System.out.println("\nTest 2 using utc: Dijkstra should be 13 and AStar should be 5");
//
//		List<GeographicPoint> testroute1 = testMap.dijkstra(testStart,testEnd);
//		System.out.println("dijkstra search. testroute list: " + testroute1);
//
//		List<GeographicPoint> testroute2 = testMap.aStarSearch(testStart,testEnd);
//		System.out.println("aStarSearch. testroute2 list: " + testroute2);


//
//
//		// A slightly more complex test using real data
//		testStart = new GeographicPoint(32.8674388, -117.2190213);
//		testEnd = new GeographicPoint(32.8697828, -117.2244506);
//		System.out.println("Test 3 using utc: Dijkstra should be 37 and AStar should be 10");
//		testroute1 = testMap.dijkstra(testStart,testEnd);
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

//		MapGraph simpleTestMap = new MapGraph();
//		GraphLoader.loadRoadMap("data/testdata/simpletest.map", simpleTestMap);
//
//		GeographicPoint testStart = new GeographicPoint(1.0, 1.0);
//		GeographicPoint testEnd = new GeographicPoint(8.0, -1.0);
//
//		System.out.println("Test 1 using simpletest: Dijkstra should be 9 and AStar should be 5");
//		List<GeographicPoint> testroute = simpleTestMap.dijkstra(testStart,testEnd);
//		List<GeographicPoint> testroute2 = simpleTestMap.aStarSearch(testStart,testEnd);
//
//
//		MapGraph testMap = new MapGraph();
//		GraphLoader.loadRoadMap("data/maps/utc.map", testMap);
//
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







		MapGraph theMap = new MapGraph();
		System.out.print("DONE. \nLoading the map...");
		GraphLoader.loadRoadMap("data/maps/utc.map", theMap);
		System.out.println("DONE.");

		GeographicPoint start = new GeographicPoint(32.8648772, -117.2254046);
		GeographicPoint end = new GeographicPoint(32.8660691, -117.217393);

		List<GeographicPoint> route = theMap.dijkstra(start,end);
		List<GeographicPoint> route2 = theMap.aStarSearch(start,end);

	}

}
