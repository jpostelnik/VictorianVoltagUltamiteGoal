package org.firstinspires.ftc.vrhsrobotics.victorianvoltage.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.vrhsrobotics.victorianvoltage.auto.vision.SkystoneDeterminationPipeline;


@Autonomous(name = "redWall")
public class redWall extends Auto {

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
            location = SkystoneDeterminationPipeline.RingPosition.NONE;
            switch (location) {
                case ONE:
                    System.out.println("b");
                    telemetry.addLine("b");
                    telemetry.update();
                    strafeByDeadWheels(1, 0.2, false, 0, runtime);
                    moveByDeadWheels(86, 0.7, 0, runtime);
                    strafeByDeadWheels(30, 0.4, true, 0, runtime);

//                    turningPID(180, 0.4, runtime);
                    dropWobble();
                    strafeByDeadWheels(5, 0.4, true, 0, runtime);
                    moveByDeadWheels(63, -0.7, 0, runtime);

                    lowerWobble();
                    sleep(1000);
                    strafeByDeadWheels(3, 0.3, false, 0, runtime);
                    raiseWobble();
                    sleep(1000);
                    moveByDeadWheels(63, 0.7, 0, runtime);
                    strafeByDeadWheels(5, 0.6, false, 0, runtime);
                    dropWobble();

                    break;
                case FOUR:
                    System.out.println("c");
                    telemetry.addLine("c");
                    telemetry.update();
                    strafeByDeadWheels(1, 0.2, false, 0, runtime);
                    moveByDeadWheels(110, 0.7, 0, runtime);
                    strafeByDeadWheels(4, 0.4, true, 0, runtime);

                    dropWobble();

                    strafeByDeadWheels(35, 0.4, true, 0, runtime);
                    moveByDeadWheels(87, -0.7, 0, runtime);

                    lowerWobble();
                    sleep(1000);
                    strafeByDeadWheels(3, 0.3, false, 0, runtime);
                    raiseWobble();
                    sleep(1000);
                    moveByDeadWheels(87, 0.7, 0, runtime);
                    strafeByDeadWheels(34, 0.6, false, 0, runtime);
                    dropWobble();
                    break;
                default:
                    // TODO: 9/29/20 add spline movement there
                    System.out.println("a");
                    telemetry.addLine("a");
                    telemetry.update();
                    moveByDeadWheels(68, 0.7, 0, runtime);
                    dropWobble();
//                    strafeByDeadWheels(35, 0.6, true, 0, runtime);
//                    moveByDeadWheels(45, -0.7, 0, runtime);
//                    lowerWobble();
//                    sleep(1000);
//                    strafeByDeadWheels(3, 0.3, false, 0, runtime);
//                    raiseWobble();
//                    sleep(1000);
//                    moveByDeadWheels(59, 0.7, 0, runtime);
//                    strafeByDeadWheels(34, 0.6, false, 0, runtime);
//                    dropWobble();
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
