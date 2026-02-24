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
