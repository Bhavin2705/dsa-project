package traffic;

public class Intersection {
    private int id;
    private String name;
    private int congestionLevel;

    public Intersection(int id, String name) {
        this.id = id;
        this.name = name;
        this.congestionLevel = 0;
    }

    public int getId() {
        return id;
    }

    public String getName() {
        return name;
    }

    public int getCongestionLevel() {
        return congestionLevel;
    }

    public void setCongestionLevel(int level) {
        this.congestionLevel = level;
    }

    public String toString() {
        return "Intersection[" + id + ": " + name + ", congestion=" + congestionLevel + "]";
    }
}
