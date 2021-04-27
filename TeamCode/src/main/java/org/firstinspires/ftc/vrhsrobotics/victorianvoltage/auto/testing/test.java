package org.firstinspires.ftc.vrhsrobotics.victorianvoltage.auto.testing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.ejml.simple.SimpleMatrix;
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

        resetRuntime();

        try {
//            strafeByDeadWheels(5, 1, false,0 , runtime);
//                        move(new SimpleMatrix(new double[][]{{0}, {50}}), 1, runtime);
//                        sleep(2000);
//                        move(new SimpleMatrix(new double[][]{{0},{-50}}),1,runtime);
////            sleep(2000);
//            move(new SimpleMatrix(new double[][]{{25}, {0}}), 1, runtime);
//            sleep(2000);
//
//            move(new SimpleMatrix(new double[][]{{-25}, {0}}), 1, runtime);
//            sleep(2000);

            move(new SimpleMatrix(new double[][]{{10}, {20}}), 1, runtime);
            sleep(2000);

            move(new SimpleMatrix(new double[][]{{-10}, {-20}}), 1, runtime);
            sleep(2000);

            move(new SimpleMatrix(new double[][]{{10}, {30}}), 1, runtime);
            sleep(2000);


            sleep(10000);
        } catch (Exception e) {
            halt();
            sleep(10 * 1000);
            e.printStackTrace();
        }
    }


}
