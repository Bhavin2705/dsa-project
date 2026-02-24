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
