package org.firstinspires.ftc.vrhsrobotics.victorianvoltage.auto.math.controltheory;

public class KalminFilter {

    /**
     * @param x this is the x degrees/distance
     * @param xHat  this is where i want to be
     * @return returns the error between this
     */
    public static double getError(double x, double xHat) {
        double e1 = xHat - x;
        double e2;
        if (x < 0) {
            e2 = xHat - (x + 360);
        } else {
            e2 = xHat - (x - 360);
        }
        if (Math.abs(e1) <= Math.abs(e2)) {
            return e1;
        }
        return e2;
    }


}
