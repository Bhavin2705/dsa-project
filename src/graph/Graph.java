package graph;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

public class Graph {
    private Map<Integer, List<Edge>> adjacencyList;
    private Map<Integer, String> nodeNames;
    private Map<Integer, Integer> nodeCongestion;
    private Map<String, Integer> edgeCongestion;
    private int nodeCount;

    public Graph() {
        adjacencyList = new HashMap<Integer, List<Edge>>();
        nodeNames = new HashMap<Integer, String>();
        nodeCongestion = new HashMap<Integer, Integer>();
        edgeCongestion = new HashMap<String, Integer>();
        nodeCount = 0;
    }

    public int addNode(String name) {
        int id = nodeCount;
        adjacencyList.put(id, new ArrayList<Edge>());
        nodeNames.put(id, name);
        nodeCongestion.put(id, 0);
        nodeCount++;
        return id;
    }

    public boolean addEdge(int from, int to, int weight) {
        if (!adjacencyList.containsKey(from) || !adjacencyList.containsKey(to)) {
            return false;
        }
        adjacencyList.get(from).add(new Edge(to, weight));
        adjacencyList.get(to).add(new Edge(from, weight));
        return true;
    }

    public void setNodeCongestion(int nodeId, int level) {
        if (nodeCongestion.containsKey(nodeId)) {
            nodeCongestion.put(nodeId, level);
            recomputeEdgeWeights(nodeId);
        }
    }

    public void setEdgeCongestion(int from, int to, int level) {
        String key = from + "_" + to;
        String reverseKey = to + "_" + from;
        edgeCongestion.put(key, level);
        edgeCongestion.put(reverseKey, level);
        recomputeEdgeWeightsBetween(from, to, level);
        recomputeEdgeWeightsBetween(to, from, level);
    }

    private void recomputeEdgeWeights(int nodeId) {
        int congestion = nodeCongestion.get(nodeId);
        List<Edge> edges = adjacencyList.get(nodeId);
        for (int i = 0; i < edges.size(); i++) {
            Edge e = edges.get(i);
            String key = nodeId + "_" + e.destination;
            int edgeCong = 0;
            if (edgeCongestion.containsKey(key)) {
                edgeCong = edgeCongestion.get(key);
            }
            e.currentWeight = e.baseWeight + congestion + edgeCong;
        }
    }

    private void recomputeEdgeWeightsBetween(int from, int to, int congestion) {
        List<Edge> edges = adjacencyList.get(from);
        if (edges == null) {
            return;
        }
        for (int i = 0; i < edges.size(); i++) {
            Edge e = edges.get(i);
            if (e.destination == to) {
                int nodeCong = 0;
                if (nodeCongestion.containsKey(from)) {
                    nodeCong = nodeCongestion.get(from);
                }
                e.currentWeight = e.baseWeight + nodeCong + congestion;
            }
        }
    }

    public int getNodeCongestion(int nodeId) {
        if (nodeCongestion.containsKey(nodeId)) {
            return nodeCongestion.get(nodeId);
        }
        return 0;
    }

    public List<Edge> getNeighbors(int node) {
        if (!adjacencyList.containsKey(node)) {
            return new ArrayList<Edge>();
        }
        return adjacencyList.get(node);
    }

    public int getNodeCount() {
        return nodeCount;
    }

    public String getNodeName(int id) {
        if (nodeNames.containsKey(id)) {
            return nodeNames.get(id);
        }
        return "Unknown";
    }

    public boolean nodeExists(int id) {
        return adjacencyList.containsKey(id);
    }

    public Map<Integer, String> getAllNodes() {
        return nodeNames;
    }

    public Map<Integer, Integer> getAllCongestion() {
        return nodeCongestion;
    }
}
