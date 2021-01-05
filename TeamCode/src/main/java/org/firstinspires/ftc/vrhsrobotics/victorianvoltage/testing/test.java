
package org.firstinspires.ftc.vrhsrobotics.victorianvoltage.testing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.vrhsrobotics.victorianvoltage.auto.Auto;


@Autonomous(name = "test")
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
            moveByDeadWheels(24,1,0,runtime);
            moveByDeadWheels(-24,1,0,runtime);
            strafeByDeadWheels(24,1,true,0,runtime);
            strafeByDeadWheels(24,1,false,0,runtime);


        } catch (InterruptedException e) {

        }

    }
}
