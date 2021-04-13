package org.firstinspires.ftc.vrhsrobotics.victorianvoltage.auto.math.controltheory;

import org.ejml.simple.SimpleMatrix;

public class Kinematic {
    private double width, height;

    private SimpleMatrix J, R;

    public Kinematic(double width, double height) {
        this.width = width;
        this.height = height;

        J = new SimpleMatrix(new double[][]{
                {-1, 1, (width + height)},
                {1, 1, -(width + height)},
                {-1, 1, -(width + height)},
                {1, 1, (width + height)}
        });
    }

//    public SimpleMatrix getPower(double Vx, double Vy, double dTheta) {
//        SimpleMatrix cx = new SimpleMatrix(new double[][]{{Vy, Vx, dTheta}});
//        SimpleMatrix power = J.mult(R).mult(cx);
//        double maxPower = power.elementMaxAbs();
//
//        return power.scale(1 / maxPower);
//    }

    // this is relative to the
    public SimpleMatrix getPowerRelative(double Vx, double Vy, double dTheta) {
        SimpleMatrix cx = new SimpleMatrix(new double[][]{{Vx, Vy, dTheta}});
        SimpleMatrix power = J.mult(cx);
        double maxPower = power.elementMaxAbs();

        return power.scale(1 / maxPower);
    }

    public void setOrientation(double theta) {
        theta = Math.toRadians(theta);
        this.R = new SimpleMatrix(new double[][]{
                {Math.cos(theta), Math.sin(theta), 0},
                {-Math.sin(theta), Math.cos(theta), 0},
                {0, 0, 1}
        });
    }

}
