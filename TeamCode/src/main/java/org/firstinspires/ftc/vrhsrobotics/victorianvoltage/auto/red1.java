package org.firstinspires.ftc.vrhsrobotics.victorianvoltage.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.vrhsrobotics.victorianvoltage.auto.vision.SkystoneDeterminationPipeline;


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
            telemetry.addData("Analysis", pipeline.getAnalysis());
            telemetry.addData("Position", pipeline.position);
            telemetry.update();
            SkystoneDeterminationPipeline.RingPosition location = pipeline.getPosition();
            close();
            //TODO: this will be were the scan method stuff will go, location 0 = a, 1 =b, 4 = c,
            //todo so if that happens will be after the shooter method
//            strafeByDeadWheels(20,0.7,true,0,runtime);
//            moveByDeadWheels(33, 0.7, 0, runtime);
//            strafeByDeadWheels(19,0.7,false,0,runtime);
//            shoot(1,5);
//            moveByDeadWheels(4,-0.5,0,runtime);
//            turningPID(-3, 1, runtime);
//            move(24, 1, 0, runtime);
////
////            shoot(1, 1);
//            // turningPID(-15,1,runtime);

            location = SkystoneDeterminationPipeline.RingPosition.ONE;
//
            switch (location) {
                case ONE:
                    System.out.println("b");
                    telemetry.addLine("b");
                    telemetry.update();
                    strafeByDeadWheels(2, 0.7, false, 0, runtime);
                    moveByDeadWheels(86, 0.7, 0, runtime);
                    strafeByDeadWheels(30, 0.4, true, 0, runtime);
//                    turningPID(180, 0.4, runtime);
                    dropWobble();
                    break;
                case FOUR:
                    System.out.println("c");
                    telemetry.addLine("c");
                    telemetry.update();
                    strafeByDeadWheels(2, 0.7, false, 0, runtime);
                    moveByDeadWheels(110, 0.7, 0, runtime);
                    strafeByDeadWheels(8, 0.4, true, 0, runtime);

                    dropWobble();

                    strafeByDeadWheels(6, 0.4, true, 0, runtime);
                    moveByDeadWheels(50,-0.7,0,runtime);
                    break;
                default:
                    // TODO: 9/29/20 add spline movement there
                    System.out.println("a");
                    telemetry.addLine("a");
                    telemetry.update();
                    moveByDeadWheels(63, 0.7, 0, runtime);
                    strafeByDeadWheels(6, 0.4, true, 0, runtime);
                    dropWobble();
                    break;
                //todo: will be what is in 0. So might remove it.
            }
            telemetry.addData("object", location);

        } catch (Exception e) {
            halt();
            turnOffIntake();
            turnOffShooter();
            close();
            telemetry.addLine(e.getStackTrace().toString());
            telemetry.update();

        }
    }
}
