#!/bin/bash

mkdir -p src/app src/graph src/traffic src/emergency src/util

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

cat > src/graph/PathCache.java << 'EOF'
package graph;

import java.util.HashMap;
import java.util.List;
import java.util.Map;

public class PathCache {
    private Map<String, List<Integer>> pathCache;
    private Map<String, Integer> distanceCache;
    private int hitCount;
    private int missCount;

    public PathCache() {
        pathCache = new HashMap<String, List<Integer>>();
        distanceCache = new HashMap<String, Integer>();
        hitCount = 0;
        missCount = 0;
    }

    private String makeKey(int source, int destination) {
        return source + "->" + destination;
    }

    public boolean hasPath(int source, int destination) {
        return pathCache.containsKey(makeKey(source, destination));
    }

    public List<Integer> getPath(int source, int destination) {
        String key = makeKey(source, destination);
        if (pathCache.containsKey(key)) {
            hitCount++;
            return pathCache.get(key);
        }
        missCount++;
        return null;
    }

    public int getDistance(int source, int destination) {
        String key = makeKey(source, destination);
        if (distanceCache.containsKey(key)) {
            return distanceCache.get(key);
        }
        return Integer.MAX_VALUE;
    }

    public void storePath(int source, int destination, List<Integer> path, int distance) {
        String key = makeKey(source, destination);
        pathCache.put(key, path);
        distanceCache.put(key, distance);
    }

    public void invalidateAll() {
        pathCache.clear();
        distanceCache.clear();
        System.out.println("Path cache cleared due to network change.");
    }

    public void invalidatePair(int source, int destination) {
        String key = makeKey(source, destination);
        pathCache.remove(key);
        distanceCache.remove(key);
    }

    public void printStats() {
        System.out.println("Cache stats -> Hits: " + hitCount + ", Misses: " + missCount + ", Stored entries: " + pathCache.size());
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
    private String previousPhase;
    private int greenDuration;
    private int yellowDuration;
    private int redDuration;
    private boolean preempted;
    private int preemptionCyclesRemaining;

    public TrafficSignal(int intersectionId, int greenDuration, int yellowDuration, int redDuration) {
        this.intersectionId = intersectionId;
        this.greenDuration = greenDuration;
        this.yellowDuration = yellowDuration;
        this.redDuration = redDuration;
        this.currentPhase = "RED";
        this.previousPhase = "RED";
        this.preempted = false;
        this.preemptionCyclesRemaining = 0;
    }

    public int getIntersectionId() {
        return intersectionId;
    }

    public String getCurrentPhase() {
        return currentPhase;
    }

    public void setCurrentPhase(String phase) {
        this.previousPhase = this.currentPhase;
        this.currentPhase = phase;
    }

    public String getPreviousPhase() {
        return previousPhase;
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

    public boolean isPreempted() {
        return preempted;
    }

    public int getPreemptionCyclesRemaining() {
        return preemptionCyclesRemaining;
    }

    public void preempt(int cycles) {
        this.previousPhase = this.currentPhase;
        this.currentPhase = "GREEN";
        this.preempted = true;
        this.preemptionCyclesRemaining = cycles;
        System.out.println("  [PREEMPTION] Signal at intersection " + intersectionId
                + " forced GREEN for " + cycles + " cycle(s).");
    }

    public void tickPreemption() {
        if (!preempted) {
            return;
        }
        preemptionCyclesRemaining--;
        if (preemptionCyclesRemaining <= 0) {
            currentPhase = previousPhase;
            preempted = false;
            System.out.println("  [PREEMPTION ENDED] Signal at intersection " + intersectionId
                    + " restored to " + currentPhase);
        }
    }

    public String toString() {
        String status = preempted ? " [PREEMPTED, " + preemptionCyclesRemaining + " cycles left]" : "";
        return "Signal[Intersection " + intersectionId + " -> " + currentPhase + status + "]";
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

            if (signal.isPreempted()) {
                System.out.println("Skipping normal cycle for preempted signal: " + signal.toString());
                signal.tickPreemption();
                signalRotationQueue.offer(id);
                continue;
            }

            System.out.println("Processing: " + signal.toString());

            String phase = signal.getCurrentPhase();
            if (phase.equals("RED")) {
                signal.setCurrentPhase("GREEN");
            } else if (phase.equals("GREEN")) {
                signal.setCurrentPhase("YELLOW");
            } else if (phase.equals("YELLOW")) {
                signal.setCurrentPhase("RED");
            }

            System.out.println("  -> After cycle: " + signal.getCurrentPhase());
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

    public void preemptSignal(int intersectionId, int cycles) {
        if (signals.containsKey(intersectionId)) {
            signals.get(intersectionId).preempt(cycles);
        }
    }

    public void forceGreen(int intersectionId) {
        if (signals.containsKey(intersectionId)) {
            preemptSignal(intersectionId, 1);
        }
    }
}
EOF

cat > src/emergency/EmergencyVehicle.java << 'EOF'
package emergency;

import java.util.List;

public class EmergencyVehicle implements Comparable<EmergencyVehicle> {
    private int id;
    private String type;
    private int priority;
    private int currentLocation;
    private int destination;
    private boolean active;
    private boolean dispatched;
    private List<Integer> assignedRoute;

    public EmergencyVehicle(int id, String type, int priority, int currentLocation, int destination) {
        this.id = id;
        this.type = type;
        this.priority = priority;
        this.currentLocation = currentLocation;
        this.destination = destination;
        this.active = true;
        this.dispatched = false;
        this.assignedRoute = null;
    }

    public int getId() {
        return id;
    }

    public String getType() {
        return type;
    }

    public int getPriority() {
        return priority;
    }

    public int getCurrentLocation() {
        return currentLocation;
    }

    public int getDestination() {
        return destination;
    }

    public boolean isActive() {
        return active;
    }

    public boolean isDispatched() {
        return dispatched;
    }

    public void setActive(boolean active) {
        this.active = active;
    }

    public void setDispatched(boolean dispatched) {
        this.dispatched = dispatched;
    }

    public void setCurrentLocation(int location) {
        this.currentLocation = location;
    }

    public List<Integer> getAssignedRoute() {
        return assignedRoute;
    }

    public void setAssignedRoute(List<Integer> route) {
        this.assignedRoute = route;
    }

    public int compareTo(EmergencyVehicle other) {
        return Integer.compare(other.priority, this.priority);
    }

    public String toString() {
        return "EmergencyVehicle[id=" + id + ", type=" + type + ", priority=" + priority
                + ", from=" + currentLocation + ", to=" + destination
                + ", active=" + active + ", dispatched=" + dispatched + "]";
    }
}
EOF

cat > src/emergency/ConflictDetector.java << 'EOF'
package emergency;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

public class ConflictDetector {

    public static List<String> detectConflicts(List<EmergencyVehicle> activeVehicles) {
        List<String> conflicts = new ArrayList<String>();
        Map<Integer, List<EmergencyVehicle>> intersectionOccupancy = new HashMap<Integer, List<EmergencyVehicle>>();

        for (int i = 0; i < activeVehicles.size(); i++) {
            EmergencyVehicle vehicle = activeVehicles.get(i);
            if (!vehicle.isActive() || vehicle.getAssignedRoute() == null) {
                continue;
            }

            List<Integer> route = vehicle.getAssignedRoute();
            for (int j = 0; j < route.size(); j++) {
                int nodeId = route.get(j);
                if (!intersectionOccupancy.containsKey(nodeId)) {
                    intersectionOccupancy.put(nodeId, new ArrayList<EmergencyVehicle>());
                }
                intersectionOccupancy.get(nodeId).add(vehicle);
            }
        }

        for (Map.Entry<Integer, List<EmergencyVehicle>> entry : intersectionOccupancy.entrySet()) {
            List<EmergencyVehicle> vehiclesAtNode = entry.getValue();
            if (vehiclesAtNode.size() > 1) {
                StringBuilder conflict = new StringBuilder();
                conflict.append("CONFLICT at intersection ").append(entry.getKey()).append(": ");
                for (int i = 0; i < vehiclesAtNode.size(); i++) {
                    conflict.append("Vehicle #").append(vehiclesAtNode.get(i).getId());
                    conflict.append("(priority=").append(vehiclesAtNode.get(i).getPriority()).append(")");
                    if (i < vehiclesAtNode.size() - 1) {
                        conflict.append(" vs ");
                    }
                }
                EmergencyVehicle winner = resolveConflict(vehiclesAtNode);
                conflict.append(" -> Precedence given to Vehicle #").append(winner.getId());
                conflicts.add(conflict.toString());
            }
        }

        return conflicts;
    }

    public static EmergencyVehicle resolveConflict(List<EmergencyVehicle> vehicles) {
        EmergencyVehicle winner = vehicles.get(0);
        for (int i = 1; i < vehicles.size(); i++) {
            if (vehicles.get(i).getPriority() > winner.getPriority()) {
                winner = vehicles.get(i);
            }
        }
        return winner;
    }

    public static void printConflicts(List<String> conflicts) {
        if (conflicts.isEmpty()) {
            System.out.println("No route conflicts detected between active emergency vehicles.");
            return;
        }
        System.out.println("\n--- Route Conflict Report ---");
        for (int i = 0; i < conflicts.size(); i++) {
            System.out.println("  " + (i + 1) + ". " + conflicts.get(i));
        }
        System.out.println("-----------------------------\n");
    }
}
EOF

cat > src/emergency/EmergencyManager.java << 'EOF'
package emergency;

import graph.Dijkstra;
import graph.Graph;
import graph.PathCache;
import traffic.SignalManager;

import java.util.ArrayList;
import java.util.List;
import java.util.PriorityQueue;

public class EmergencyManager {
    private PriorityQueue<EmergencyVehicle> vehicleQueue;
    private List<EmergencyVehicle> allVehicles;
    private List<EmergencyVehicle> activeDispatched;
    private int nextId;
    private Graph graph;
    private Dijkstra dijkstra;
    private SignalManager signalManager;
    private PathCache pathCache;
    private static final int PREEMPTION_CYCLES = 3;

    public EmergencyManager(Graph graph, SignalManager signalManager, PathCache pathCache) {
        this.graph = graph;
        this.signalManager = signalManager;
        this.pathCache = pathCache;
        this.dijkstra = new Dijkstra(graph);
        this.vehicleQueue = new PriorityQueue<EmergencyVehicle>();
        this.allVehicles = new ArrayList<EmergencyVehicle>();
        this.activeDispatched = new ArrayList<EmergencyVehicle>();
        this.nextId = 1;
    }

    public void registerVehicle(String type, int priority, int from, int destination) {
        EmergencyVehicle vehicle = new EmergencyVehicle(nextId, type, priority, from, destination);
        nextId++;
        vehicleQueue.offer(vehicle);
        allVehicles.add(vehicle);
        System.out.println("Registered: " + vehicle.toString());
    }

    public void dispatchNextVehicle() {
        if (vehicleQueue.isEmpty()) {
            System.out.println("No emergency vehicles in queue.");
            return;
        }

        EmergencyVehicle vehicle = vehicleQueue.poll();
        System.out.println("\n--- Dispatching Emergency Vehicle ---");
        System.out.println("Vehicle: " + vehicle.toString());

        int from = vehicle.getCurrentLocation();
        int to = vehicle.getDestination();

        List<Integer> path = null;
        int distance = Integer.MAX_VALUE;
        boolean fromCache = false;

        if (pathCache.hasPath(from, to)) {
            path = pathCache.getPath(from, to);
            distance = pathCache.getDistance(from, to);
            fromCache = true;
            System.out.println("[CACHE HIT] Using cached route for " + from + " -> " + to);
        } else {
            path = dijkstra.getShortestPath(from, to);
            distance = dijkstra.getShortestDistance(from, to);
            if (!path.isEmpty()) {
                pathCache.storePath(from, to, path, distance);
                System.out.println("[CACHE MISS] Computed and stored new route for " + from + " -> " + to);
            }
        }

        if (path == null || path.isEmpty()) {
            System.out.println("No route found from " + graph.getNodeName(from) + " to " + graph.getNodeName(to));
        } else {
            System.out.println("Optimal Route (congestion-aware):");
            for (int i = 0; i < path.size(); i++) {
                int nodeId = path.get(i);
                System.out.print("  " + graph.getNodeName(nodeId));
                if (i < path.size() - 1) {
                    System.out.print(" -> ");
                }

                if (signalManager.hasSignal(nodeId)) {
                    signalManager.preemptSignal(nodeId, PREEMPTION_CYCLES);
                }
            }
            System.out.println();
            System.out.println("Total congestion-adjusted distance: " + distance + " units");
            System.out.println("Source used: " + (fromCache ? "CACHE" : "DIJKSTRA COMPUTATION"));

            vehicle.setAssignedRoute(path);
            vehicle.setDispatched(true);
            activeDispatched.add(vehicle);

            runConflictCheck();
        }

        pathCache.printStats();
        System.out.println("--- Dispatch Complete ---\n");
    }

    private void runConflictCheck() {
        List<String> conflicts = ConflictDetector.detectConflicts(activeDispatched);
        ConflictDetector.printConflicts(conflicts);
    }

    public void checkAllConflicts() {
        System.out.println("\n--- Manual Conflict Check ---");
        runConflictCheck();
    }

    public void clearCache() {
        pathCache.invalidateAll();
    }

    public void displayQueue() {
        if (vehicleQueue.isEmpty()) {
            System.out.println("No vehicles in emergency queue.");
            return;
        }

        PriorityQueue<EmergencyVehicle> copy = new PriorityQueue<EmergencyVehicle>(vehicleQueue);
        System.out.println("\n--- Emergency Vehicle Queue (by priority) ---");
        int rank = 1;
        while (!copy.isEmpty()) {
            EmergencyVehicle v = copy.poll();
            System.out.println("  " + rank + ". " + v.toString());
            rank++;
        }
        System.out.println("---------------------------------------------\n");
    }

    public void displayAllVehicles() {
        if (allVehicles.isEmpty()) {
            System.out.println("No vehicles registered.");
            return;
        }
        System.out.println("\n--- All Registered Emergency Vehicles ---");
        for (EmergencyVehicle v : allVehicles) {
            System.out.println("  " + v.toString());
        }
        System.out.println("-----------------------------------------\n");
    }

    public int getQueueSize() {
        return vehicleQueue.size();
    }

    public List<EmergencyVehicle> getActiveDispatched() {
        return activeDispatched;
    }
}
EOF

cat > src/util/SimulationClock.java << 'EOF'
package util;

public class SimulationClock {
    private int currentTick;
    private int signalCycleInterval;
    private int ticksPerHour;

    public SimulationClock(int signalCycleInterval, int ticksPerHour) {
        this.currentTick = 0;
        this.signalCycleInterval = signalCycleInterval;
        this.ticksPerHour = ticksPerHour;
    }

    public void advance(int ticks) {
        for (int i = 0; i < ticks; i++) {
            currentTick++;
        }
        System.out.println("[CLOCK] Advanced " + ticks + " tick(s). Current tick: " + currentTick
                + " | Simulated time: " + getFormattedTime());
    }

    public void advanceOne() {
        currentTick++;
    }

    public int getCurrentTick() {
        return currentTick;
    }

    public boolean isSignalCycleDue() {
        return (currentTick % signalCycleInterval) == 0 && currentTick > 0;
    }

    public String getFormattedTime() {
        int totalMinutes = (currentTick * 60) / ticksPerHour;
        int hours = totalMinutes / 60;
        int minutes = totalMinutes % 60;
        return String.format("%02d:%02d", hours % 24, minutes);
    }

    public void printStatus() {
        System.out.println("\n[SIMULATION CLOCK]");
        System.out.println("  Current Tick     : " + currentTick);
        System.out.println("  Simulated Time   : " + getFormattedTime());
        System.out.println("  Signal Cycle Due : " + isSignalCycleDue());
        System.out.println("  Cycle Interval   : every " + signalCycleInterval + " tick(s)");
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

import emergency.EmergencyManager;
import graph.Graph;
import graph.PathCache;
import traffic.SignalManager;
import util.InputUtil;
import util.SimulationClock;

import java.util.Map;

public class Main {
    private static Graph graph = new Graph();
    private static SignalManager signalManager = new SignalManager();
    private static PathCache pathCache = new PathCache();
    private static EmergencyManager emergencyManager = new EmergencyManager(graph, signalManager, pathCache);
    private static SimulationClock clock = new SimulationClock(2, 12);
    private static InputUtil input = new InputUtil();

    public static void main(String[] args) {
        System.out.println("========================================");
        System.out.println("  Traffic Management & Emergency Route  ");
        System.out.println("         Optimization System v2         ");
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
                handleRegisterEmergencyVehicle();
            } else if (choice == 6) {
                handleDispatchEmergencyVehicle();
            } else if (choice == 7) {
                handleViewQueue();
            } else if (choice == 8) {
                handleViewNetwork();
            } else if (choice == 9) {
                handleViewAllVehicles();
            } else if (choice == 10) {
                handleSetNodeCongestion();
            } else if (choice == 11) {
                handleSetEdgeCongestion();
            } else if (choice == 12) {
                handleAdvanceTime();
            } else if (choice == 13) {
                handleViewClock();
            } else if (choice == 14) {
                handleConflictCheck();
            } else if (choice == 15) {
                handleCacheStats();
            } else if (choice == 16) {
                handleClearCache();
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
        System.out.println("  1.  Add Intersection");
        System.out.println("  2.  Add Road between Intersections");
        System.out.println("  3.  Add Traffic Signal to Intersection");
        System.out.println(" --- Congestion ---");
        System.out.println("  10. Set Intersection Congestion");
        System.out.println("  11. Set Road (Edge) Congestion");
        System.out.println(" --- Simulation ---");
        System.out.println("  4.  Run Signal Cycle");
        System.out.println("  12. Advance Simulation Time");
        System.out.println("  13. View Simulation Clock");
        System.out.println(" --- Emergency ---");
        System.out.println("  5.  Register Emergency Vehicle");
        System.out.println("  6.  Dispatch Highest Priority Vehicle");
        System.out.println("  14. Check Route Conflicts");
        System.out.println(" --- Info & Cache ---");
        System.out.println("  7.  View Emergency Vehicle Queue");
        System.out.println("  8.  View Road Network");
        System.out.println("  9.  View All Vehicles");
        System.out.println("  15. View Path Cache Stats");
        System.out.println("  16. Clear Path Cache");
        System.out.println(" --- System ---");
        System.out.println("  0.  Exit");
        System.out.println("=======================================");
    }

    private static void handleAddIntersection() {
        String name = input.readString("Enter intersection name: ");
        if (name.isEmpty()) {
            System.out.println("Name cannot be empty.");
            return;
        }
        int id = graph.addNode(name);
        pathCache.invalidateAll();
        System.out.println("Intersection added with ID: " + id + " -> " + name);
        clock.advanceOne();
        System.out.println("[CLOCK] Tick advanced to " + clock.getCurrentTick());
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
            pathCache.invalidateAll();
            System.out.println("Road added between " + graph.getNodeName(from)
                    + " and " + graph.getNodeName(to) + " with base distance " + weight);
            clock.advanceOne();
            System.out.println("[CLOCK] Tick advanced to " + clock.getCurrentTick());
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
        clock.advance(2);
        System.out.println("[CLOCK] Signal cycle advanced time.");
    }

    private static void handleRegisterEmergencyVehicle() {
        if (graph.getNodeCount() < 2) {
            System.out.println("Need at least 2 intersections in the network.");
            return;
        }
        displayIntersections();
        System.out.println("Vehicle types: AMBULANCE, FIRE_TRUCK, POLICE");
        String type = input.readString("Enter vehicle type: ");
        int priority = input.readInt("Enter priority (higher = more urgent, e.g. 1-10): ");
        int from = input.readInt("Enter current location (intersection ID): ");
        int to = input.readInt("Enter destination (intersection ID): ");

        if (!graph.nodeExists(from) || !graph.nodeExists(to)) {
            System.out.println("Invalid intersection IDs.");
            return;
        }

        if (from == to) {
            System.out.println("Source and destination cannot be the same.");
            return;
        }

        emergencyManager.registerVehicle(type, priority, from, to);
    }

    private static void handleDispatchEmergencyVehicle() {
        if (emergencyManager.getQueueSize() == 0) {
            System.out.println("No vehicles in the queue.");
            return;
        }
        emergencyManager.dispatchNextVehicle();
        clock.advance(3);
        System.out.println("[CLOCK] Emergency dispatch advanced simulation time.");

        if (clock.isSignalCycleDue()) {
            System.out.println("[AUTO] Signal cycle interval reached. Running automatic signal cycle.");
            signalManager.simulateOneCycle();
        }
    }

    private static void handleViewQueue() {
        emergencyManager.displayQueue();
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
            System.out.print("  [" + id + "] " + name + " (node congestion=" + congestion + ") -> ");

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

    private static void handleViewAllVehicles() {
        emergencyManager.displayAllVehicles();
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
        pathCache.invalidateAll();
        System.out.println("Congestion at " + graph.getNodeName(id) + " set to " + level);
        System.out.println("All cached paths invalidated due to network weight change.");
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
        pathCache.invalidateAll();
        System.out.println("Road congestion between " + graph.getNodeName(from)
                + " and " + graph.getNodeName(to) + " set to " + level);
        System.out.println("All cached paths invalidated.");
    }

    private static void handleAdvanceTime() {
        int ticks = input.readInt("Enter number of ticks to advance: ");
        if (ticks <= 0) {
            System.out.println("Must advance at least 1 tick.");
            return;
        }
        clock.advance(ticks);
        if (clock.isSignalCycleDue()) {
            System.out.println("[AUTO] Signal cycle interval reached during time advance.");
            signalManager.simulateOneCycle();
        }
    }

    private static void handleViewClock() {
        clock.printStatus();
    }

    private static void handleConflictCheck() {
        emergencyManager.checkAllConflicts();
    }

    private static void handleCacheStats() {
        pathCache.printStats();
    }

    private static void handleClearCache() {
        pathCache.invalidateAll();
        System.out.println("Cache cleared manually.");
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
            System.out.println("  ID: " + entry.getKey() + " -> " + entry.getValue() + " (congestion: " + cong + ")");
        }
    }
}
EOF

echo ""
echo "Project created successfully with all features."
echo ""
echo "New features included:"
echo "  1. Congestion-Aware Routing   - node + edge congestion adjusts Dijkstra weights"
echo "  2. Temporary Signal Preemption- forced GREEN for fixed cycles, then auto-restore"
echo "  3. Conflict Detection         - route overlap detection with priority resolution"
echo "  4. Shortest Path Caching      - source->dest cache with hit/miss stats"
echo "  5. Simulation Clock           - global tick counter with auto signal triggering"
echo ""
echo "To compile and run:"
echo "  cd src"
echo "  javac app/Main.java graph/Edge.java graph/Graph.java graph/Dijkstra.java graph/PathCache.java traffic/Intersection.java traffic/TrafficSignal.java traffic/SignalManager.java emergency/EmergencyVehicle.java emergency/ConflictDetector.java emergency/EmergencyManager.java util/InputUtil.java util/SimulationClock.java"
echo "  java app.Main"