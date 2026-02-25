#!/bin/bash
mkdir -p src/app src/graph src/traffic src/util
cat > src/graph/Edge.java << 'EOF'
package graph;
public class Edge {
    public int destination;
    public int baseWeight;
    public int currentWeight;
    public Edge(int destination, int weight) {
        this.destination = destination;
        this.baseWeight = weight;
        this.currentWeight = weight;
    }
    public void updateCurrentWeight(int congestion) {
        this.currentWeight = this.baseWeight + congestion;
    }
}
EOF
cat > src/graph/Graph.java << 'EOF'
package graph;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
public class Graph {
    private Map<Integer, List<Edge>> adjacencyList;
    private Map<Integer, String> nodeNames;
    private Map<Integer, Integer> nodeCongestion;
    private Map<String, Integer> edgeCongestion;
    private int nodeCount;
    public Graph() {
        adjacencyList = new HashMap<Integer, List<Edge>>();
        nodeNames = new HashMap<Integer, String>();
        nodeCongestion = new HashMap<Integer, Integer>();
        edgeCongestion = new HashMap<String, Integer>();
        nodeCount = 0;
    }
    public int addNode(String name) {
        int id = nodeCount;
        adjacencyList.put(id, new ArrayList<Edge>());
        nodeNames.put(id, name);
        nodeCongestion.put(id, 0);
        nodeCount++;
        return id;
    }
    public boolean addEdge(int from, int to, int weight) {
        if (!adjacencyList.containsKey(from) || !adjacencyList.containsKey(to)) {
            return false;
        }
        adjacencyList.get(from).add(new Edge(to, weight));
        adjacencyList.get(to).add(new Edge(from, weight));
        return true;
    }
    public void setNodeCongestion(int nodeId, int level) {
        if (nodeCongestion.containsKey(nodeId)) {
            nodeCongestion.put(nodeId, level);
            recomputeEdgeWeights(nodeId);
        }
    }
    public void setEdgeCongestion(int from, int to, int level) {
        String key = from + "_" + to;
        String reverseKey = to + "_" + from;
        edgeCongestion.put(key, level);
        edgeCongestion.put(reverseKey, level);
        recomputeEdgeWeightsBetween(from, to, level);
        recomputeEdgeWeightsBetween(to, from, level);
    }
    private void recomputeEdgeWeights(int nodeId) {
        int congestion = nodeCongestion.get(nodeId);
        List<Edge> edges = adjacencyList.get(nodeId);
        for (int i = 0; i < edges.size(); i++) {
            Edge e = edges.get(i);
            String key = nodeId + "_" + e.destination;
            int edgeCong = 0;
            if (edgeCongestion.containsKey(key)) {
                edgeCong = edgeCongestion.get(key);
            }
            e.currentWeight = e.baseWeight + congestion + edgeCong;
        }
    }
    private void recomputeEdgeWeightsBetween(int from, int to, int congestion) {
        List<Edge> edges = adjacencyList.get(from);
        if (edges == null) {
            return;
        }
        for (int i = 0; i < edges.size(); i++) {
            Edge e = edges.get(i);
            if (e.destination == to) {
                int nodeCong = 0;
                if (nodeCongestion.containsKey(from)) {
                    nodeCong = nodeCongestion.get(from);
                }
                e.currentWeight = e.baseWeight + nodeCong + congestion;
            }
        }
    }
    public int getNodeCongestion(int nodeId) {
        if (nodeCongestion.containsKey(nodeId)) {
            return nodeCongestion.get(nodeId);
        }
        return 0;
    }
    public List<Edge> getNeighbors(int node) {
        if (!adjacencyList.containsKey(node)) {
            return new ArrayList<Edge>();
        }
        return adjacencyList.get(node);
    }
    public int getNodeCount() {
        return nodeCount;
    }
    public String getNodeName(int id) {
        if (nodeNames.containsKey(id)) {
            return nodeNames.get(id);
        }
        return "Unknown";
    }
    public boolean nodeExists(int id) {
        return adjacencyList.containsKey(id);
    }
    public Map<Integer, String> getAllNodes() {
        return nodeNames;
    }
    public Map<Integer, Integer> getAllCongestion() {
        return nodeCongestion;
    }
}
EOF
cat > src/graph/Dijkstra.java << 'EOF'
package graph;
import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.List;
import java.util.PriorityQueue;
public class Dijkstra {
    private Graph graph;
    public Dijkstra(Graph graph) {
        this.graph = graph;
    }
    public int[] computeDistances(int source) {
        int n = graph.getNodeCount();
        int[] distances = new int[n];
        for (int i = 0; i < n; i++) {
            distances[i] = Integer.MAX_VALUE;
        }
        distances[source] = 0;
        PriorityQueue<int[]> pq = new PriorityQueue<int[]>(n, new Comparator<int[]>() {
            public int compare(int[] a, int[] b) {
                return Integer.compare(a[1], b[1]);
            }
        });
        pq.offer(new int[]{source, 0});
        while (!pq.isEmpty()) {
            int[] current = pq.poll();
            int node = current[0];
            int dist = current[1];
            if (dist > distances[node]) {
                continue;
            }
            for (Edge edge : graph.getNeighbors(node)) {
                if (distances[node] == Integer.MAX_VALUE) {
                    continue;
                }
                int newDist = distances[node] + edge.currentWeight;
                if (newDist < distances[edge.destination]) {
                    distances[edge.destination] = newDist;
                    pq.offer(new int[]{edge.destination, newDist});
                }
            }
        }
        return distances;
    }
    public int[] computePrevious(int source) {
        int n = graph.getNodeCount();
        int[] distances = new int[n];
        int[] previous = new int[n];
        for (int i = 0; i < n; i++) {
            distances[i] = Integer.MAX_VALUE;
            previous[i] = -1;
        }
        distances[source] = 0;
        PriorityQueue<int[]> pq = new PriorityQueue<int[]>(n, new Comparator<int[]>() {
            public int compare(int[] a, int[] b) {
                return Integer.compare(a[1], b[1]);
            }
        });
        pq.offer(new int[]{source, 0});
        while (!pq.isEmpty()) {
            int[] current = pq.poll();
            int node = current[0];
            int dist = current[1];
            if (dist > distances[node]) {
                continue;
            }
            for (Edge edge : graph.getNeighbors(node)) {
                if (distances[node] == Integer.MAX_VALUE) {
                    continue;
                }
                int newDist = distances[node] + edge.currentWeight;
                if (newDist < distances[edge.destination]) {
                    distances[edge.destination] = newDist;
                    previous[edge.destination] = node;
                    pq.offer(new int[]{edge.destination, newDist});
                }
            }
        }
        return previous;
    }
    public List<Integer> getShortestPath(int source, int destination) {
        int[] previous = computePrevious(source);
        int[] distances = computeDistances(source);
        List<Integer> path = new ArrayList<Integer>();
        if (destination >= distances.length || distances[destination] == Integer.MAX_VALUE) {
            return path;
        }
        int current = destination;
        while (current != -1) {
            path.add(current);
            current = previous[current];
        }
        Collections.reverse(path);
        return path;
    }
    public int getShortestDistance(int source, int destination) {
        int[] distances = computeDistances(source);
        if (destination < distances.length) {
            return distances[destination];
        }
        return Integer.MAX_VALUE;
    }
}
EOF
cat > src/traffic/Intersection.java << 'EOF'
package traffic;
public class Intersection {
    private int id;
    private String name;
    private int congestionLevel;
    public Intersection(int id, String name) {
        this.id = id;
        this.name = name;
        this.congestionLevel = 0;
    }
    public int getId() {
        return id;
    }
    public String getName() {
        return name;
    }
    public int getCongestionLevel() {
        return congestionLevel;
    }
    public void setCongestionLevel(int level) {
        this.congestionLevel = level;
    }
    public String toString() {
        return "Intersection[" + id + ": " + name + ", congestion=" + congestionLevel + "]";
    }
}
EOF
cat > src/traffic/TrafficSignal.java << 'EOF'
package traffic;
public class TrafficSignal {
    private int intersectionId;
    private String currentPhase;
    private int greenDuration;
    private int yellowDuration;
    private int redDuration;
    public TrafficSignal(int intersectionId, int greenDuration, int yellowDuration, int redDuration) {
        this.intersectionId = intersectionId;
        this.greenDuration = greenDuration;
        this.yellowDuration = yellowDuration;
        this.redDuration = redDuration;
        this.currentPhase = "RED";
    }
    public int getIntersectionId() {
        return intersectionId;
    }
    public String getCurrentPhase() {
        return currentPhase;
    }
    public void setCurrentPhase(String phase) {
        this.currentPhase = phase;
    }
    public int getGreenDuration() {
        return greenDuration;
    }
    public int getYellowDuration() {
        return yellowDuration;
    }
    public int getRedDuration() {
        return redDuration;
    }
    public String toString() {
        return "Signal[Intersection " + intersectionId + " -> " + currentPhase + "]";
    }
}
EOF
cat > src/traffic/SignalManager.java << 'EOF'
package traffic;
import java.util.HashMap;
import java.util.LinkedList;
import java.util.Map;
import java.util.Queue;
public class SignalManager {
    private Map<Integer, TrafficSignal> signals;
    private Queue<Integer> signalRotationQueue;
    public SignalManager() {
        signals = new HashMap<Integer, TrafficSignal>();
        signalRotationQueue = new LinkedList<Integer>();
    }
    public void addSignal(int intersectionId, int green, int yellow, int red) {
        TrafficSignal signal = new TrafficSignal(intersectionId, green, yellow, red);
        signals.put(intersectionId, signal);
        signalRotationQueue.offer(intersectionId);
    }
    public void simulateOneCycle() {
        if (signalRotationQueue.isEmpty()) {
            System.out.println("No signals to simulate.");
            return;
        }
        int size = signalRotationQueue.size();
        System.out.println("\n--- Traffic Signal Simulation Cycle ---");
        for (int i = 0; i < size; i++) {
            int id = signalRotationQueue.poll();
            TrafficSignal signal = signals.get(id);
            System.out.println("Processing: " + signal.toString());
            String phase = signal.getCurrentPhase();
            if (phase.equals("RED")) {
                signal.setCurrentPhase("GREEN");
            } else if (phase.equals("GREEN")) {
                signal.setCurrentPhase("YELLOW");
            } else if (phase.equals("YELLOW")) {
                signal.setCurrentPhase("RED");
            }
            System.out.println(" -> After cycle: " + signal.getCurrentPhase());
            signalRotationQueue.offer(id);
        }
        System.out.println("--- End of Cycle ---\n");
    }
    public void displayAllSignals() {
        if (signals.isEmpty()) {
            System.out.println("No signals registered.");
            return;
        }
        System.out.println("\n--- Current Signal States ---");
        for (Map.Entry<Integer, TrafficSignal> entry : signals.entrySet()) {
            System.out.println(entry.getValue().toString());
        }
        System.out.println("-----------------------------\n");
    }
    public boolean hasSignal(int intersectionId) {
        return signals.containsKey(intersectionId);
    }
    public TrafficSignal getSignal(int intersectionId) {
        return signals.get(intersectionId);
    }
}
EOF
cat > src/util/InputUtil.java << 'EOF'
package util;
import java.util.Scanner;
public class InputUtil {
    private Scanner scanner;
    public InputUtil() {
        scanner = new Scanner(System.in);
    }
    public int readInt(String prompt) {
        System.out.print(prompt);
        while (!scanner.hasNextInt()) {
            System.out.println("Please enter a valid integer.");
            scanner.next();
            System.out.print(prompt);
        }
        int value = scanner.nextInt();
        scanner.nextLine();
        return value;
    }
    public String readString(String prompt) {
        System.out.print(prompt);
        return scanner.nextLine().trim();
    }
    public void close() {
        scanner.close();
    }
}
EOF
cat > src/app/Main.java << 'EOF'
package app;
import graph.Dijkstra;
import graph.Graph;
import traffic.SignalManager;
import util.InputUtil;
import java.util.List;
import java.util.Map;
public class Main {
    private static Graph graph = new Graph();
    private static SignalManager signalManager = new SignalManager();
    private static Dijkstra dijkstra = new Dijkstra(graph);
    private static InputUtil input = new InputUtil();
    public static void main(String[] args) {
        System.out.println("========================================");
        System.out.println(" Traffic Management & Route ");
        System.out.println(" Optimization System ");
        System.out.println("========================================");
        boolean running = true;
        while (running) {
            printMainMenu();
            int choice = input.readInt("Enter choice: ");
            if (choice == 1) {
                handleAddIntersection();
            } else if (choice == 2) {
                handleAddRoad();
            } else if (choice == 3) {
                handleAddSignal();
            } else if (choice == 4) {
                handleSimulateSignals();
            } else if (choice == 5) {
                handleFindShortestPath();
            } else if (choice == 6) {
                handleSetNodeCongestion();
            } else if (choice == 7) {
                handleSetEdgeCongestion();
            } else if (choice == 8) {
                handleViewNetwork();
            } else if (choice == 0) {
                System.out.println("Exiting system. Goodbye.");
                running = false;
            } else {
                System.out.println("Invalid option. Try again.");
            }
        }
        input.close();
    }
    private static void printMainMenu() {
        System.out.println("\n============== MAIN MENU ==============");
        System.out.println(" --- Network Setup ---");
        System.out.println(" 1. Add Intersection");
        System.out.println(" 2. Add Road between Intersections");
        System.out.println(" 3. Add Traffic Signal to Intersection");
        System.out.println(" --- Congestion ---");
        System.out.println(" 6. Set Intersection Congestion");
        System.out.println(" 7. Set Road (Edge) Congestion");
        System.out.println(" --- Routing ---");
        System.out.println(" 5. Find Shortest Path");
        System.out.println(" --- Simulation ---");
        System.out.println(" 4. Run Signal Cycle");
        System.out.println(" --- Info ---");
        System.out.println(" 8. View Road Network");
        System.out.println(" --- System ---");
        System.out.println(" 0. Exit");
        System.out.println("=======================================");
    }
    private static void handleAddIntersection() {
        String name = input.readString("Enter intersection name: ");
        if (name.isEmpty()) {
            System.out.println("Name cannot be empty.");
            return;
        }
        int id = graph.addNode(name);
        System.out.println("Intersection added with ID: " + id + " -> " + name);
    }
    private static void handleAddRoad() {
        if (graph.getNodeCount() < 2) {
            System.out.println("Need at least 2 intersections to add a road.");
            return;
        }
        displayIntersections();
        int from = input.readInt("Enter source intersection ID: ");
        int to = input.readInt("Enter destination intersection ID: ");
        int weight = input.readInt("Enter road base distance/weight: ");
        if (weight <= 0) {
            System.out.println("Weight must be positive.");
            return;
        }
        boolean added = graph.addEdge(from, to, weight);
        if (added) {
            System.out.println("Road added between " + graph.getNodeName(from)
                    + " and " + graph.getNodeName(to) + " with base distance " + weight);
        } else {
            System.out.println("Failed to add road. Check intersection IDs.");
        }
    }
    private static void handleAddSignal() {
        if (graph.getNodeCount() == 0) {
            System.out.println("No intersections available.");
            return;
        }
        displayIntersections();
        int id = input.readInt("Enter intersection ID for signal: ");
        if (!graph.nodeExists(id)) {
            System.out.println("Intersection not found.");
            return;
        }
        if (signalManager.hasSignal(id)) {
            System.out.println("Signal already exists at this intersection.");
            return;
        }
        int green = input.readInt("Enter green phase duration (seconds): ");
        int yellow = input.readInt("Enter yellow phase duration (seconds): ");
        int red = input.readInt("Enter red phase duration (seconds): ");
        signalManager.addSignal(id, green, yellow, red);
        System.out.println("Traffic signal added at intersection: " + graph.getNodeName(id));
    }
    private static void handleSimulateSignals() {
        signalManager.simulateOneCycle();
        signalManager.displayAllSignals();
    }
    private static void handleFindShortestPath() {
        if (graph.getNodeCount() < 2) {
            System.out.println("Need at least 2 intersections in the network.");
            return;
        }
        displayIntersections();
        int from = input.readInt("Enter source intersection ID: ");
        int to = input.readInt("Enter destination intersection ID: ");
        if (!graph.nodeExists(from) || !graph.nodeExists(to)) {
            System.out.println("Invalid intersection IDs.");
            return;
        }
        if (from == to) {
            System.out.println("Source and destination cannot be the same.");
            return;
        }
        List<Integer> path = dijkstra.getShortestPath(from, to);
        int distance = dijkstra.getShortestDistance(from, to);
        if (path.isEmpty()) {
            System.out.println("No route found from " + graph.getNodeName(from) + " to " + graph.getNodeName(to));
            return;
        }
        System.out.println("\nShortest path (congestion-aware):");
        for (int i = 0; i < path.size(); i++) {
            int nodeId = path.get(i);
            System.out.print(" " + graph.getNodeName(nodeId));
            if (i < path.size() - 1) {
                System.out.print(" -> ");
            }
        }
        System.out.println();
        System.out.println("Total congestion-adjusted distance: " + distance + " units");
    }
    private static void handleSetNodeCongestion() {
        if (graph.getNodeCount() == 0) {
            System.out.println("No intersections available.");
            return;
        }
        displayIntersections();
        int id = input.readInt("Enter intersection ID: ");
        if (!graph.nodeExists(id)) {
            System.out.println("Intersection not found.");
            return;
        }
        int level = input.readInt("Enter congestion level (0 = clear, higher = more congested): ");
        if (level < 0) {
            System.out.println("Congestion level cannot be negative.");
            return;
        }
        graph.setNodeCongestion(id, level);
        System.out.println("Congestion at " + graph.getNodeName(id) + " set to " + level);
    }
    private static void handleSetEdgeCongestion() {
        if (graph.getNodeCount() < 2) {
            System.out.println("Need at least 2 intersections.");
            return;
        }
        displayIntersections();
        int from = input.readInt("Enter source intersection ID: ");
        int to = input.readInt("Enter destination intersection ID: ");
        if (!graph.nodeExists(from) || !graph.nodeExists(to)) {
            System.out.println("Invalid intersection IDs.");
            return;
        }
        int level = input.readInt("Enter road congestion level (0 = clear): ");
        if (level < 0) {
            System.out.println("Congestion level cannot be negative.");
            return;
        }
        graph.setEdgeCongestion(from, to, level);
        System.out.println("Road congestion between " + graph.getNodeName(from)
                + " and " + graph.getNodeName(to) + " set to " + level);
    }
    private static void handleViewNetwork() {
        Map<Integer, String> nodes = graph.getAllNodes();
        if (nodes.isEmpty()) {
            System.out.println("No intersections in the network.");
            return;
        }
        System.out.println("\n--- Road Network (Congestion-Aware) ---");
        for (Map.Entry<Integer, String> entry : nodes.entrySet()) {
            int id = entry.getKey();
            String name = entry.getValue();
            int congestion = graph.getNodeCongestion(id);
            System.out.print(" [" + id + "] " + name + " (node congestion=" + congestion + ") -> ");
            java.util.List<graph.Edge> neighbors = graph.getNeighbors(id);
            if (neighbors.isEmpty()) {
                System.out.print("(no roads)");
            } else {
                for (int i = 0; i < neighbors.size(); i++) {
                    graph.Edge e = neighbors.get(i);
                    System.out.print(graph.getNodeName(e.destination)
                            + "(base:" + e.baseWeight + ", effective:" + e.currentWeight + ")");
                    if (i < neighbors.size() - 1) {
                        System.out.print(", ");
                    }
                }
            }
            System.out.println();
        }
        System.out.println("---------------------------------------\n");
        signalManager.displayAllSignals();
    }
    private static void displayIntersections() {
        Map<Integer, String> nodes = graph.getAllNodes();
        if (nodes.isEmpty()) {
            System.out.println("No intersections available.");
            return;
        }
        System.out.println("Available intersections:");
        for (Map.Entry<Integer, String> entry : nodes.entrySet()) {
            int cong = graph.getNodeCongestion(entry.getKey());
            System.out.println(" ID: " + entry.getKey() + " -> " + entry.getValue() + " (congestion: " + cong + ")");
        }
    }
}
EOF
echo ""
echo "Project created successfully."
echo ""
echo "To compile and run:"
echo " cd src"
echo " javac app/Main.java graph/Edge.java graph/Graph.java graph/Dijkstra.java traffic/Intersection.java traffic/TrafficSignal.java traffic/SignalManager.java util/InputUtil.java"
echo " java app.Main"