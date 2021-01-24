package org.firstinspires.ftc.vrhsrobotics.victorianvoltage.auto.math.controltheory;

public class KalminFilter {

    private final double TICKS_PER_REV = 560;
    private final double DRIVETRAIN_WHEEL_DIAMTER = 4;
    private final double DRIVETRAIN_GEAR_RATIO = 1;
    private final double LINEAR_TO_TICKS = TICKS_PER_REV / (Math.PI * DRIVETRAIN_WHEEL_DIAMTER * DRIVETRAIN_GEAR_RATIO);
    private final double radius = 6.5;
    private final double STRAFE_COEFFICIENT = 1.20;
    private final double DEAD_WHEEL_DIAMTER = 2;
    private final double DEAD_WHEEL_TICKS_PER_REV = 4096;
    private final double DEAD_WHEEL_TO_TICKS = DEAD_WHEEL_TICKS_PER_REV / (Math.PI * DEAD_WHEEL_DIAMTER);




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

    public double distanceTraveled(double leftFront, double rightFront, double leftRear, double rightRear, double deadWheelLet, double deadWheelRight )
    {
        double distance = 0;

        double avgMotors = (leftFront+rightFront+rightRear+leftRear)/4/LINEAR_TO_TICKS;
        double avgDeadWheel = (deadWheelLet+deadWheelRight)/2/DEAD_WHEEL_TO_TICKS;

        distance = avgMotors*0.4+avgDeadWheel*0.6;

        return distance*DEAD_WHEEL_TO_TICKS;
    }


}
