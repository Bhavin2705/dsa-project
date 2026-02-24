package graph;

import java.util.HashMap;
import java.util.List;
import java.util.Map;

public class PathCache {
    private Map<String, List<Integer>> pathCache;
    private Map<String, Integer> distanceCache;
    private int hitCount;
    private int missCount;

    public PathCache() {
        pathCache = new HashMap<String, List<Integer>>();
        distanceCache = new HashMap<String, Integer>();
        hitCount = 0;
        missCount = 0;
    }

    private String makeKey(int source, int destination) {
        return source + "->" + destination;
    }

    public boolean hasPath(int source, int destination) {
        return pathCache.containsKey(makeKey(source, destination));
    }

    public List<Integer> getPath(int source, int destination) {
        String key = makeKey(source, destination);
        if (pathCache.containsKey(key)) {
            hitCount++;
            return pathCache.get(key);
        }
        missCount++;
        return null;
    }

    public int getDistance(int source, int destination) {
        String key = makeKey(source, destination);
        if (distanceCache.containsKey(key)) {
            return distanceCache.get(key);
        }
        return Integer.MAX_VALUE;
    }

    public void storePath(int source, int destination, List<Integer> path, int distance) {
        String key = makeKey(source, destination);
        pathCache.put(key, path);
        distanceCache.put(key, distance);
    }

    public void invalidateAll() {
        pathCache.clear();
        distanceCache.clear();
        System.out.println("Path cache cleared due to network change.");
    }

    public void invalidatePair(int source, int destination) {
        String key = makeKey(source, destination);
        pathCache.remove(key);
        distanceCache.remove(key);
    }

    public void printStats() {
        System.out.println("Cache stats -> Hits: " + hitCount + ", Misses: " + missCount + ", Stored entries: " + pathCache.size());
    }
}
