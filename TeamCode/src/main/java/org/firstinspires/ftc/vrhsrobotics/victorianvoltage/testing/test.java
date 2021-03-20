
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
            strafeByDeadWheels(3, 0.3, false, 0, runtime);
            shoot(0.8, 1);
            strafeByDeadWheels(3, 0.3, false, 0, runtime);
            shoot(0.8, 1);
            strafeByDeadWheels(3, 0.3, false, 0, runtime);
//            sleep(10000);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }

    public void powerShot(ElapsedTime runtime) throws InterruptedException {




    }


}
