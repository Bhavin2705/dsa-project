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
