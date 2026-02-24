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
