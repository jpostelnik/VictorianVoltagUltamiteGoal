package org.firstinspires.ftc.vrhsrobotics.victorianvoltage.auto.testing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.vrhsrobotics.victorianvoltage.auto.*;
import org.firstinspires.ftc.vrhsrobotics.victorianvoltage.auto.math.spline.Spline;

@Autonomous(name = "spline testing")
@Disabled
public class    splineTest extends Auto {

    private ElapsedTime runtime =new ElapsedTime();

    @Override
    public void runOpMode() {

        double[] xCords = {0,-10, -9, -3};
        double[] yCords = {0,5,15,20};
        Spline spline = new Spline(xCords,yCords);
        double arclen = spline.getArcLen();
        telemetry.addLine("starting to initialize");
        telemetry.update();
        initialize();

        waitForStart();
        runtime.reset();
        resetRuntime();

        try{
            spline(spline,arclen, 0, 0.4);
        } catch (Exception e) {
        }


    }

}
