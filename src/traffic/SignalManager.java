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
