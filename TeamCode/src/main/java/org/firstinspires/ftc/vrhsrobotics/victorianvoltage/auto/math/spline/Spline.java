package org.firstinspires.ftc.vrhsrobotics.victorianvoltage.auto.math.spline;

public class Spline {
    private Waypoint[] cords;
    private double[] xCords, yCords;
    private final int CURVE_POINTS = 3;

    public Spline(Waypoint[] cords) {
        this.cords = cords;

        for(int i = 0; i < cords.length; i++) {
            xCords[i] = cords[i].getXcoord();
            yCords[i] = cords[i].getYcoord();
        }
    }

    public double getXLength(double t)
    {
        return Math.pow(1-t,3)*xCords[0]+ 3*Math.pow(1-t,2)*xCords[1]*t+3*Math.pow(t,2)*xCords[2]*(1-t)*Math.pow(t,3)*xCords[3];
    }


    public double getYLength(double t)
    {
        return Math.pow(1-t,3)*yCords[0]+ 3*Math.pow(1-t,2)*yCords[1]*t+3*Math.pow(t,2)*yCords[2]*(1-t)*Math.pow(t,3)*yCords[3];
    }
}
