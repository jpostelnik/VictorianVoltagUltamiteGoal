 package org.firstinspires.ftc.vrhsrobotics.victorianvoltage.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.vrhsrobotics.victorianvoltage.auto.vision.SkystoneDeterminationPipeline;


@Autonomous(name = "redWall4WheelsTest")
public class redWall4WheelsTest extends Auto {

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
            SkystoneDeterminationPipeline.RingPosition location = pipeline.position;
            close();
            //TODO:: Get one and four to work
            //TODO:: try not to cry

//            location = SkystoneDeterminationPipeline.RingPosition.NONE;
            switch (location) {
                case ONE:
                    System.out.println("b");
                    telemetry.addLine("b");
                    telemetry.update();
                    strafeByDeadWheels(4, 0.2, false, 0, runtime);
                    moveByDeadWheels(90, 0.7, 0, runtime);
                    strafeByDeadWheels(25, 0.4, true, 0, runtime);

//                    turningPID(180, 0.4, runtime);
                    dropWobble();
                    strafeByDeadWheels(20, 0.4, true, 0, runtime);
                    moveByDeadWheels(65, -0.7, 0, runtime);

                    lowerWobble();
                    sleep(1000);
                    strafeByDeadWheels(3, 0.3, false, 0, runtime);
                    raiseWobble();
                    sleep(1000);
                    moveByDeadWheels(71, 0.7, 0, runtime);
                    strafeByDeadWheels(10, 0.6, false, 0, runtime);
                    dropWobble();
                    moveByDeadWheels(15, -0.7, 0, runtime);
                    // strafeByDeadWheels(5, 0.6, true, 0, runtime);
                    // shoot(0.8, 1);
                    break;
                case FOUR:
                    System.out.println("c");
                    telemetry.addLine("c");
                    telemetry.update();
                    strafeByDeadWheels(6, 0.3, false, 0, runtime);
                    moveByDeadWheels(110, 0.8, 0, runtime);
//                    strafeByDeadWheels(5, 0.3, true, 0, runtime);

                    dropWobble();

                    strafeByDeadWheels(41, 0.3, true, 0, runtime);
                    moveByDeadWheels(87, -0.8, 0, runtime);

                    lowerWobble();
                    sleep(1000);
                    strafeByDeadWheels(3, 0.3, false, 0, runtime);
                    raiseWobble();
                    sleep(1000);
                    moveByDeadWheels(96, 0.8, 0, runtime);
                    strafeByDeadWheels(30, 0.3, false, 0, runtime);
                    dropWobble();
                    moveByDeadWheels(30, -0.8, 0, runtime);
                    //strafeByDeadWheels(5, 0.6, true, 0, runtime);
                    //shoot(0.8, 1);
                    break;
                default:
                    System.out.println("c");
                    telemetry.addLine("c");
                    telemetry.update();
                    strafeByDeadWheels(4, 0.3, false, 0, runtime);
                    moveByDeadWheels(104, 0.8, 0, runtime);
//                    strafeByDeadWheels(5, 0.3, true, 0, runtime);

                    dropWobble();

                    strafeByDeadWheels(40, 0.8, true, 0, runtime);
                    moveByDeadWheels(85, -0.8, 0, runtime);

                    lowerWobble();
                    sleep(1000);
                    strafeByDeadWheels(5, 0.3, false, 0, runtime);
                    raiseWobble();
                    sleep(1000);
                    moveByDeadWheels(87, 0.8, 0, runtime);
                    strafeByDeadWheels(30, 0.3, false, 0, runtime);
                    dropWobble();
                    moveByDeadWheels(30, -0.8, 0, runtime);
                    //strafeByDeadWheels(5, 0.6, true, 0, runtime);
                    //shoot(0.8, 1);
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