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
