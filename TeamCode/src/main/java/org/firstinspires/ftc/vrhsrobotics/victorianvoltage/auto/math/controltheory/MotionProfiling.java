package org.firstinspires.ftc.vrhsrobotics.victorianvoltage.auto.math.controltheory;

import org.ejml.simple.SimpleMatrix;

public class MotionProfiling {
    private double currentPower, maxPower;
    private SimpleMatrix target;
    private Kinematic kinematic;

    private static final double STEP = 0.05;
    private static final double INITIAL = 0.03;


    public MotionProfiling(double maxPower, Kinematic kinematic, SimpleMatrix target) {
        currentPower = INITIAL;
        this.kinematic = kinematic;
        this.maxPower = maxPower;
        this.target = target;
    }

    public double power() {
        currentPower += STEP;
        return currentPower > maxPower ? maxPower : currentPower;

    }

    public SimpleMatrix power(SimpleMatrix error) {
        double percent = target.normF() / error.normF();
        SimpleMatrix unit = error.scale(1 / error.normF());
        if(percent<0.25){
            currentPower += STEP;
        }else if(percent>0.75){
            currentPower -= STEP;
        }else {
            currentPower = maxPower;
        }
        return kinematic.getPowerRelative(unit.get(0) * currentPower, unit.get(1) * currentPower, 0);
    }


}
