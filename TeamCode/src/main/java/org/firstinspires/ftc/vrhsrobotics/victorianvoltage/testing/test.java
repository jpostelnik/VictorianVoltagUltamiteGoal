
package org.firstinspires.ftc.vrhsrobotics.victorianvoltage.testing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.vrhsrobotics.victorianvoltage.auto.Auto;


@Autonomous(name = "test")
@Disabled
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
            resetDeadWheels();
            moveByTime(1,0.5,runtime);
//            printTicks();
            sleep(30000);

        } catch (InterruptedException e) {

        }

    }
}
