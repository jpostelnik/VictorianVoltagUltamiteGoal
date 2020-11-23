package org.firstinspires.ftc.vrhsrobotics.victorianvoltage.testing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.vrhsrobotics.victorianvoltage.auto.*;

@Autonomous(name = "spline testing")
public class splineTest extends Auto {

    private ElapsedTime runtime =new ElapsedTime();

    @Override
    public void runOpMode() {

        telemetry.addLine("starting to initialize");
        telemetry.update();
        initialize();
        double[] xCords = {0,5, 15, 20};
        double[] yCords = {0,10, 9, 3};
        waitForStart();
        runtime.reset();
        restRuntime();

        try{
            spline(xCords, yCords, 0, 1);
        } catch (InterruptedException e) {
        }


    }

}
