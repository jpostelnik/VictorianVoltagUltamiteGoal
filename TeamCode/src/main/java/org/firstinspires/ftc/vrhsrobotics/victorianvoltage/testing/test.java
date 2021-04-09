package org.firstinspires.ftc.vrhsrobotics.victorianvoltage.testing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.vrhsrobotics.victorianvoltage.auto.Auto;


@Autonomous(name = "test")
//@Disabled
public class test extends Auto {

    /***
     * this runs the auto class
     */

    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {

        telemetry.addLine("starting to initialize");
        telemetry.update();
        initialize();
        waitForStart();
        runtime.reset();

        restRuntime();

        try {
//            strafeByDeadWheels(5, 1, false,0 , runtime);
            move(50, 1, 0, runtime);
            sleep(10000);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }


}
