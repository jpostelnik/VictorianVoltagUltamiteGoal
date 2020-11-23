package org.firstinspires.ftc.vrhsrobotics.victorianvoltage.auto.math.spline;

public class Spline {
    private double[] xCords, yCords;
    private final int CURVE_POINTS = 3;
    private final double DEAD_WHEEL_TICKS_PER_REV = 4096;


    public Spline(double[] xCords, double[] yCords) {
        this.xCords = xCords;
        this.yCords = yCords;
    }

    public double getXLength(double t)
    {
        double a = (Math.pow((1-t),3)*xCords[0]*DEAD_WHEEL_TICKS_PER_REV)+3*(Math.pow((1-t),2)*xCords[1]*DEAD_WHEEL_TICKS_PER_REV)+3*((1-t)*xCords[2]*DEAD_WHEEL_TICKS_PER_REV*Math.pow(t,2))+(Math.pow(t,3)*xCords[3]*DEAD_WHEEL_TICKS_PER_REV);
        System.out.println("x = "+a+" @t ="+t);
        return a;
    }


    public double getYLength(double t)
    {
        double a =(Math.pow((1-t),3)*yCords[0]*DEAD_WHEEL_TICKS_PER_REV)+3*(Math.pow((1-t),2)*yCords[1]*DEAD_WHEEL_TICKS_PER_REV)+3*((1-t)*yCords[2]*DEAD_WHEEL_TICKS_PER_REV*Math.pow(t,2))+(Math.pow(t,3)*yCords[3]*DEAD_WHEEL_TICKS_PER_REV);
        System.out.println("y = "+a+" @t ="+t);
        return a;
    }

    public double arcLength()
    {
        double length = 0;
        for (double i = 0; i <=1; i+=0.01) {
            System.out.println("i = "+i);
           length +=  Math.sqrt(Math.pow(getXLength(i),2)+Math.pow(getYLength(i),2))*0.01;
           System.out.println(length);
        }

        return length;
    }


    @Override
    public String toString() {
        return ""+arcLength();
    }


}
