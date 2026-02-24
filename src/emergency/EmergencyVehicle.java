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
