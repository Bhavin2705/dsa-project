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
