package org.firstinspires.ftc.vrhsrobotics.victorianvoltage.testing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.vrhsrobotics.victorianvoltage.auto.Auto;

@Autonomous(name = "battery plotting")
@Disabled
public class batteryPlotting extends Auto {
    @Override
    public void runOpMode() {
        ElapsedTime runtime = new ElapsedTime();
        initialize();
        waitForStart();
        runtime.reset();
        try {
            moveByTime(2, 1, runtime);
        } catch (InterruptedException e) {

        }

    }
}
