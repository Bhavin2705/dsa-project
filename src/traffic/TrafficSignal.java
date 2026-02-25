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
