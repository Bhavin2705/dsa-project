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
