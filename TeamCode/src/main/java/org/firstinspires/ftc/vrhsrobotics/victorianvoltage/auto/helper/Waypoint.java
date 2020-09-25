package auto.helper;

public class Waypoint {
    private double xCord, yCord;
    public Waypoint(double x, double y)
    {
        xCord = x;
        yCord = y;
    }
    public double getXcoord()
    {
        return xCord;
    }
    public double getYcoord()
    {
        return yCord;
    }
    public String toString()
    {
        return "x: " + xCord + ", y: " + yCord;
    }
}
