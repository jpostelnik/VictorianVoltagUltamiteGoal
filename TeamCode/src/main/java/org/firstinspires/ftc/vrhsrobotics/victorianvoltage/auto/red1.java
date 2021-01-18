package org.firstinspires.ftc.vrhsrobotics.victorianvoltage.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;


@Autonomous(name = "redWall")
public class red1 extends Auto {

    public void runOpMode() {
        initialize();
        telemetry.addLine("all initialized");
        telemetry.update();
        ElapsedTime runtime = new ElapsedTime();
        waitForStart();
        runtime.reset();
        try {
            //TODO: this will be were the scan method stuff will go, location 0 = a, 1 =b, 4 = c,
            //todo so if that happens will be after the shooter method
            moveByDeadWheels(10, 1, 0, runtime);
            strafeByDeadWheels(8,1,false,0,runtime);
//            turningPID(20, 0.5, runtime);
            sleep(1000);
            String location = getObjectAmount();
//            turningPID(-3, 1, runtime);
//            move(24, 1, 0, runtime);
////
////            shoot(1, 1);
//            // turningPID(-15,1,runtime);
//
//            switch (location) {
//                case "Single":
//                    moveByDeadWheels(18, 1, 0, runtime);
//                    System.out.println("b");
//                    telemetry.addLine("b");
//                    telemetry.update();
//                    break;
//                case "Quad":
//                    moveByDeadWheels(36, 1, 0, runtime);
//
//                    System.out.println("c");
//                    telemetry.addLine("c");
//                    telemetry.update();
//                    break;
//
//                default:
//                    turningPID(180, 0.4, runtime);
//                    // TODO: 9/29/20 add spline movement there
//                    System.out.println("a");
//                    telemetry.addLine("a");
//                    telemetry.update();
////                    turningPID(30, 1, runtime);
//                    break;
//                //todo: will be what is in 0. So might remove it.
//            }
            sleep(10000);

        } catch (Exception e) {
            telemetry.addLine(e.getStackTrace().toString());
            telemetry.update();
        }
    }
}
