/**
 * @author UCSD MOOC development team and YOU
 * 
 * A class which reprsents a graph of geographic locations
 * Nodes in the graph are intersections between 
 *
 */
package roadgraph;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.Comparator;
import java.util.HashMap;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.PriorityQueue;
import java.util.Set;
import java.util.function.Consumer;

import geography.GeographicPoint;
import util.GraphLoader;

/**
 * @author UCSD MOOC development team and YOU
 * 
 *         A class which represents a graph of geographic locations Nodes in the
 *         graph are intersections between
 *
 */
public class MapGraph {
	// TODO: Add your member variables here in WEEK 2
	private int numVertices;
	private int numEdges;
	private Set<GeographicPoint> points;
	private HashMap<GeographicPoint, ArrayList<MapEdge>> edges;

	/**
	 * Create a new empty MapGraph
	 */
	public MapGraph() {
		// TODO: Implement in this constructor in WEEK 2
		numEdges = 0;
		numVertices = 0;
		points = new HashSet<>();
		edges = new HashMap<>();
	}

	/**
	 * Get the number of vertices (road intersections) in the graph
	 * 
	 * @return The number of vertices in the graph.
	 */
	public int getNumVertices() {
		// TODO: Implement this method in WEEK 2
		return numVertices;
	}

	/**
	 * Return the intersections, which are the vertices in this graph.
	 * 
	 * @return The vertices in this graph as GeographicPoints
	 */
	public Set<GeographicPoint> getVertices() {
		// TODO: Implement this method in WEEK 2
		return points;
	}

	/**
	 * Get the number of road segments in the graph
	 * 
	 * @return The number of edges in the graph.
	 */
	public int getNumEdges() {
		// TODO: Implement this method in WEEK 2
		return numEdges;
	}

	/**
	 * Add a node corresponding to an intersection at a Geographic Point If the
	 * location is already in the graph or null, this method does not change the
	 * graph.
	 * 
	 * @param location
	 *            The location of the intersection
	 * @return true if a node was added, false if it was not (the node was
	 *         already in the graph, or the parameter is null).
	 */
	public boolean addVertex(GeographicPoint location) {
		// TODO: Implement this method in WEEK 2
		if (location == null)
			return false;
		if (points.contains(location))
			return false;
		numVertices++;
		return points.add(location);
	}

	/**
	 * Adds a directed edge to the graph from pt1 to pt2. Precondition: Both
	 * GeographicPoints have already been added to the graph
	 * 
	 * @param from
	 *            The starting point of the edge
	 * @param to
	 *            The ending point of the edge
	 * @param roadName
	 *            The name of the road
	 * @param roadType
	 *            The type of the road
	 * @param length
	 *            The length of the road, in km
	 * @throws IllegalArgumentException
	 *             If the points have not already been added as nodes to the
	 *             graph, if any of the arguments is null, or if the length is
	 *             less than 0.
	 */
	public void addEdge(GeographicPoint from, GeographicPoint to, String roadName, String roadType, double length)
			throws IllegalArgumentException {

		// TODO: Implement this method in WEEK 2
		if (!points.contains(from) || !points.contains(to))
			throw new IllegalArgumentException("non existent vertices");
		if (length < 0)
			throw new IllegalArgumentException("invalid length");
		if (!edges.containsKey(from))
			edges.put(from, new ArrayList<MapEdge>());
		// System.out.println(from + " " + to + " " + length);
		edges.get(from).add(new MapEdge(to, roadName, roadType, length));
		numEdges++;
	}

	private void printGraph() {
		for (GeographicPoint entry : edges.keySet()) {
			System.out.print(entry + " ==> ");
			for (MapEdge adj : edges.get(entry)) {
				System.out.print(adj + " --> ");
			}
			System.out.println();
		}

	}

	/**
	 * Find the path from start to goal using breadth first search
	 * 
	 * @param start
	 *            The starting location
	 * @param goal
	 *            The goal location
	 * @return The list of intersections that form the shortest (unweighted)
	 *         path from start to goal (including both start and goal).
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
	 * @param start
	 *            The starting location
	 * @param goal
	 *            The goal location
	 * @param nodeSearched
	 *            A hook for visualization. See assignment instructions for how
	 *            to use it.
	 * @return The list of intersections that form the shortest (unweighted)
	 *         path from start to goal (including both start and goal).
	 */
	public List<GeographicPoint> bfs(GeographicPoint start, GeographicPoint goal,
			Consumer<GeographicPoint> nodeSearched) {
		// TODO: Implement this method in WEEK 2

		// Hook for visualization. See writeup.
		// nodeSearched.accept(next.getLocation());
		Queue<GeographicPoint> queue = new Queue<>();
		Set<GeographicPoint> visited = new HashSet<>();
		HashMap<GeographicPoint, GeographicPoint> parent = new HashMap<>();
		/* Running BFS */
		if (doBfs(queue, visited, parent, start, goal, nodeSearched))
			return constructPath(start, goal, parent);
		else
			return null;
	}

	private boolean doBfs(Queue<GeographicPoint> queue, Set<GeographicPoint> visited,
			HashMap<GeographicPoint, GeographicPoint> parent, GeographicPoint start, GeographicPoint goal,
			Consumer<GeographicPoint> nodeSearched) {
		/* Enqueue the first node */
		queue.enQueue(start);
		visited.add(start);
		int count = 0;
		/* Run till the queue is empty */
		while (!queue.isEmpty()) {
			GeographicPoint loc = queue.deQueue();
			/* update the consumer as nodes are explored */
			nodeSearched.accept(loc);
			/* no need to continue if the vertex has no edges */
			if (edges.get(loc) == null) {
				continue;
			}
			for (MapEdge node : edges.get(loc)) {
				/* if vertex isn't explored yet, update the parent */
				if (!visited.contains(node.getDst())) {
					queue.enQueue(node.getDst());
					visited.add(node.getDst());
					parent.put(node.getDst(), loc);
					/* stop if goal is found */
					if (node.getDst().equals(goal)) {
						return true;
					}
				}
			}
		}
		return false;
	}

	private ArrayList<GeographicPoint> constructPath(GeographicPoint start, GeographicPoint goal,
			HashMap<GeographicPoint, GeographicPoint> parent) {
		List<GeographicPoint> path = new ArrayList<>();
		GeographicPoint loc = goal;
		/* start with the goal and back track till the start is found */
		path.add(goal);
		// System.out.println("WORKING ON " + start + " " + goal + " ");
		// System.out.println(parent.keySet());
		// for (GeographicPoint gg : parent.keySet()) {
		// System.out.println(gg + " " + parent.get(gg));
		// }
		while (!loc.equals(start)) {
			path.add(parent.get(loc));
			loc = parent.get(loc);
			// System.out.println("After" + parent.get(loc));
		}
		/* reversing the list for the expected format */
		Collections.reverse(path);
		return (ArrayList<GeographicPoint>) path;
	}

	/**
	 * Find the path from start to goal using Dijkstra's algorithm
	 * 
	 * @param start
	 *            The starting location
	 * @param goal
	 *            The goal location
	 * @return The list of intersections that form the shortest path from start
	 *         to goal (including both start and goal).
	 */
	public List<GeographicPoint> dijkstra(GeographicPoint start, GeographicPoint goal) {
		// Dummy variable for calling the search algorithms
		// You do not need to change this method.
		Consumer<GeographicPoint> temp = (x) -> {
		};
		return dijkstra(start, goal, temp);
	}

	/**
	 * Find the path from start to goal using Dijkstra's algorithm
	 * 
	 * @param start
	 *            The starting location
	 * @param goal
	 *            The goal location
	 * @param nodeSearched
	 *            A hook for visualization. See assignment instructions for how
	 *            to use it.
	 * @return The list of intersections that form the shortest path from start
	 *         to goal (including both start and goal).
	 */
	public List<GeographicPoint> dijkstra(GeographicPoint start, GeographicPoint goal,
			Consumer<GeographicPoint> nodeSearched) {
		// TODO: Implement this method in WEEK 3

		HashMap<GeographicPoint, GeographicPoint> parent = new HashMap<>();
		/* Map to store the distances */
		HashMap<GeographicPoint, Double> distMap = new HashMap<>();

		for (GeographicPoint node : points) {
			distMap.put(node, Double.MAX_VALUE);
			parent.put(node, null);
		}
		distMap.put(start, 0.0);
		return doDijkstra(start, goal, nodeSearched, parent, distMap);

	}

	private List<GeographicPoint> doDijkstra(GeographicPoint start, GeographicPoint goal,
			Consumer<GeographicPoint> nodeSearched, HashMap<GeographicPoint, GeographicPoint> parent,
			HashMap<GeographicPoint, Double> distMap) {

		Comparator<GeographicPoint> distComp = new Comparator<GeographicPoint>() {

			@Override
			public int compare(GeographicPoint o1, GeographicPoint o2) {
				if (distMap.get(o1) < distMap.get(o2))
					return -1;
				if (distMap.get(o1) > distMap.get(o2))
					return 1;
				return 0;
			}
		};

		PriorityQueue<GeographicPoint> queue = new PriorityQueue<>(distComp);
		// HashSet<GeographicPoint> visited = new HashSet<>();
		// for (GeographicPoint node : points)
		// queue.add(node);
		queue.add(start);
		int count = 0;
		while (!queue.isEmpty()) {
			// System.out.println("QUEUE" + queue);
			GeographicPoint loc = queue.remove();
			count += 1;
			System.out.println("HERE");
			// System.out.println("loc " + loc);
			// System.out.println("AFTER " + queue);
			// Hook for visualization. See writeup.
			nodeSearched.accept(loc);
			if (loc.equals(goal)) {
//				System.out.println("GOAL FOUND");
//				System.out.println(start + " " + goal + " " + parent);
			System.out.println("DIJ COUNT " + count);
				return constructPath(start, goal, parent);
			}
			if (edges.get(loc) == null)
				continue;
			for (MapEdge node : edges.get(loc)) {
				if (distMap.get(node.getDst()) > distMap.get(loc) + node.getLength()) {
					distMap.put(node.getDst(), distMap.get(loc) + node.getLength());
					// System.out.println("put " + loc);
					// queue.remove(node.getDst());
					queue.add(node.getDst());
					parent.put(node.getDst(), loc);
				}
			}
			// System.out.println("MAP after " + distMap);

		}
		return null;
	}

	/**
	 * Find the path from start to goal using A-Star search
	 * 
	 * @param start
	 *            The starting location
	 * @param goal
	 *            The goal location
	 * @return The list of intersections that form the shortest path from start
	 *         to goal (including both start and goal).
	 */
	public List<GeographicPoint> aStarSearch(GeographicPoint start, GeographicPoint goal) {
		// Dummy variable for calling the search algorithms
		Consumer<GeographicPoint> temp = (x) -> {
		};
		return aStarSearch(start, goal, temp);
	}

	/**
	 * Find the path from start to goal using A-Star search
	 * 
	 * @param start
	 *            The starting location
	 * @param goal
	 *            The goal location
	 * @param nodeSearched
	 *            A hook for visualization. See assignment instructions for how
	 *            to use it.
	 * @return The list of intersections that form the shortest path from start
	 *         to goal (including both start and goal).
	 */
	public List<GeographicPoint> aStarSearch(GeographicPoint start, GeographicPoint goal,
			Consumer<GeographicPoint> nodeSearched) {
		// TODO: Implement this method in WEEK 3

		// Hook for visualization. See writeup.
		// nodeSearched.accept(next.getLocation());

		HashMap<GeographicPoint, GeographicPoint> parent = new HashMap<>();
		/* Map to store the distances */
		HashMap<GeographicPoint, Double> distMap = new HashMap<>();

		for (GeographicPoint node : points) {
			distMap.put(node, Double.MAX_VALUE);
			parent.put(node, null);
		}
		distMap.put(start, 0.0);
		return doAstar(start, goal, nodeSearched, parent, distMap);

	}

	private List<GeographicPoint> doAstar(GeographicPoint start, GeographicPoint goal,
			Consumer<GeographicPoint> nodeSearched, HashMap<GeographicPoint, GeographicPoint> parent,
			HashMap<GeographicPoint, Double> distMap) {

		Comparator<GeographicPoint> distComp = new Comparator<GeographicPoint>() {

			@Override
			public int compare(GeographicPoint o1, GeographicPoint o2) {
				if ((distMap.get(o1) + heuristic(o1, goal)) < distMap.get(o2) + heuristic(o2, goal))
					return -1;
				if ((distMap.get(o1) + heuristic(o1, goal)) > distMap.get(o2) + heuristic(o2, goal))
					return 1;
				return 0;
			}
		};

		PriorityQueue<GeographicPoint> queue = new PriorityQueue<>(distComp);
		// HashSet<GeographicPoint> visited = new HashSet<>();
		// for (GeographicPoint node : points)
		// queue.add(node);
		queue.add(start);
		int count = 0;
		while (!queue.isEmpty()) {
			// System.out.println("QUEUE" + queue);
			GeographicPoint loc = queue.remove();
			count++;
			// System.out.println("loc " + loc);
			// System.out.println("AFTER " + queue);
			// Hook for visualization. See writeup.
//			System.out.println("Visited " + loc);
			nodeSearched.accept(loc);
			if (loc.equals(goal)) {
//				System.out.println("GOAL FOUND");
//				System.out.println(start + " " + goal + " " + parent);
				System.out.println("AStar COUNT " + count);
				return constructPath(start, goal, parent);
			}
			if (edges.get(loc) == null)
				continue;
			for (MapEdge node : edges.get(loc)) {
				if (distMap.get(node.getDst()) > distMap.get(loc) + node.getLength()) {
					distMap.put(node.getDst(), distMap.get(loc) + node.getLength());
					// System.out.println("put " + loc);
					// queue.remove(node.getDst());
//					queue.add(node.getDst() + heuristic(node.getDst(), goal));
					queue.add(node.getDst());
					parent.put(node.getDst(), loc);
				}
			}
			// System.out.println("MAP after " + distMap);

		}
		return null;
	}

	private double heuristic(GeographicPoint start, GeographicPoint goal) {
//		Double out = new Double(start.getX() - goal.getX()) + (start.getY() - goal.getY());
//		System.out.println("Heuristic " + start + goal + " " + out);
		return start.distance(goal);
	}

	public static void main(String[] args) {
		System.out.print("Making a new map...");
//		MapGraph theMap = new MapGraph();
//		System.out.print("DONE. \nLoading the map...");
//		GraphLoader.loadRoadMap("data/testdata/simpletest.map", theMap);
//		theMap.printGraph();
//		System.out.println("DONE.");
//		System.out.println(theMap.aStarSearch((GeographicPoint) theMap.points.toArray()[0],
//				(GeographicPoint) theMap.points.toArray()[5]));

		// You can use this method for testing.

		/*
		 * Use this code in Week 3 End of Week Quiz 
		 */ 
			MapGraph theMap = new MapGraph(); 
			System.out.print("DONE. \nLoading the map...");
		  GraphLoader.loadRoadMap("data/maps/utc.map", theMap);
		  System.out.println("DONE.");
		  
		  GeographicPoint start = new GeographicPoint(32.8648772, -117.2254046);
		  GeographicPoint end = new GeographicPoint(32.8660691, -117.217393);
		  
//		  GeographicPoint start = new GeographicPoint(32.868629, -117.215393);
//		  GeographicPoint end = new GeographicPoint(32.868629, -117.215393);
		  
		  List<GeographicPoint> route = theMap.dijkstra(start,end);
		  System.out.println(route);
		  List<GeographicPoint> route2 = theMap.aStarSearch(start,end);

	}

}
