package graph;
import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.List;
import java.util.PriorityQueue;
public class Dijkstra {
    private Graph graph;
    public Dijkstra(Graph graph) {
        this.graph = graph;
    }
    public int[] computeDistances(int source) {
        int n = graph.getNodeCount();
        int[] distances = new int[n];
        for (int i = 0; i < n; i++) {
            distances[i] = Integer.MAX_VALUE;
        }
        distances[source] = 0;
        PriorityQueue<int[]> pq = new PriorityQueue<int[]>(n, new Comparator<int[]>() {
            public int compare(int[] a, int[] b) {
                return Integer.compare(a[1], b[1]);
            }
        });
        pq.offer(new int[]{source, 0});
        while (!pq.isEmpty()) {
            int[] current = pq.poll();
            int node = current[0];
            int dist = current[1];
            if (dist > distances[node]) {
                continue;
            }
            for (Edge edge : graph.getNeighbors(node)) {
                if (distances[node] == Integer.MAX_VALUE) {
                    continue;
                }
                int newDist = distances[node] + edge.currentWeight;
                if (newDist < distances[edge.destination]) {
                    distances[edge.destination] = newDist;
                    pq.offer(new int[]{edge.destination, newDist});
                }
            }
        }
        return distances;
    }
    public int[] computePrevious(int source) {
        int n = graph.getNodeCount();
        int[] distances = new int[n];
        int[] previous = new int[n];
        for (int i = 0; i < n; i++) {
            distances[i] = Integer.MAX_VALUE;
            previous[i] = -1;
        }
        distances[source] = 0;
        PriorityQueue<int[]> pq = new PriorityQueue<int[]>(n, new Comparator<int[]>() {
            public int compare(int[] a, int[] b) {
                return Integer.compare(a[1], b[1]);
            }
        });
        pq.offer(new int[]{source, 0});
        while (!pq.isEmpty()) {
            int[] current = pq.poll();
            int node = current[0];
            int dist = current[1];
            if (dist > distances[node]) {
                continue;
            }
            for (Edge edge : graph.getNeighbors(node)) {
                if (distances[node] == Integer.MAX_VALUE) {
                    continue;
                }
                int newDist = distances[node] + edge.currentWeight;
                if (newDist < distances[edge.destination]) {
                    distances[edge.destination] = newDist;
                    previous[edge.destination] = node;
                    pq.offer(new int[]{edge.destination, newDist});
                }
            }
        }
        return previous;
    }
    public List<Integer> getShortestPath(int source, int destination) {
        int[] previous = computePrevious(source);
        int[] distances = computeDistances(source);
        List<Integer> path = new ArrayList<Integer>();
        if (destination >= distances.length || distances[destination] == Integer.MAX_VALUE) {
            return path;
        }
        int current = destination;
        while (current != -1) {
            path.add(current);
            current = previous[current];
        }
        Collections.reverse(path);
        return path;
    }
    public int getShortestDistance(int source, int destination) {
        int[] distances = computeDistances(source);
        if (destination < distances.length) {
            return distances[destination];
        }
        return Integer.MAX_VALUE;
    }
}
