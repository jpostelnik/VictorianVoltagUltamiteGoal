package org.firstinspires.ftc.vrhsrobotics.victorianvoltage.auto.math.controltheory;

import org.ejml.simple.SimpleMatrix;
import com.qualcomm.robotcore.util.Range;

public class MotionProfiling {
    private double currentPower, maxPower;
    private SimpleMatrix target;
    private Kinematic kinematic;

    private static final double STEP = 0.05;
    private static final double MIN_POWER = 0.15;

    public MotionProfiling(double maxPower, Kinematic kinematic, SimpleMatrix target) {
        currentPower = MIN_POWER;
        this.kinematic = kinematic;
        this.maxPower = maxPower;
        this.target = target;
    }

    public double power() {
        currentPower += STEP;

        return currentPower > maxPower ? maxPower : currentPower;
    }

    public double power(dir SimpleMatrix, percent double) {
        if (percent < 0.25) {
            currentPower = Range.clip(currentPower + STEP, MIN_POWER, maxPower);
        } else if (percent > 0.75) {
            currentPower = Range.clip(currentPower - STEP, MIN_POWER, maxPower);
        } else {
            currentPower = maxPower;
        }

        return currentPower;
    }


}
