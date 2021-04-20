package org.firstinspires.ftc.vrhsrobotics.victorianvoltage.auto.redauto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.vrhsrobotics.victorianvoltage.auto.Auto;
import org.firstinspires.ftc.vrhsrobotics.victorianvoltage.auto.exceptions.HeartBeatException;
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
            SkystoneDeterminationPipeline.RingPosition location = pipeline.position;
            close();
            //TODO:: Get one and four to work
            //TODO:: try not to cry

            location = SkystoneDeterminationPipeline.RingPosition.ONE;
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
                    strafeByDeadWheels(23, 0.4, true, 0, runtime);
                    moveByDeadWheels(70, -0.7, 0, runtime);

                    lowerWobble();
                    sleep(1000);
                    strafeByDeadWheels(3, 0.3, false, 0, runtime);
                    raiseWobble();
                    sleep(1000);
                    moveByDeadWheels(71, 0.7, 0, runtime);
                    strafeByDeadWheels(10, 0.6, false, 0, runtime);
                    dropWobble();
                    moveByDeadWheels(20, -0.7, 0, runtime);
                     strafeByDeadWheels(5, 0.6, true, 0, runtime);
                     shoot(1, 1,4);
                    break;
                case FOUR:
                    System.out.println("c");
                    telemetry.addLine("c");
                    telemetry.update();
                    strafeByDeadWheels(4, 0.3, false, 0, runtime);
                    moveByDeadWheels(110, 0.8, 0, runtime);

                    dropWobble();

                    strafeByDeadWheels(43, 0.8, true, 0, runtime);
                    moveByDeadWheels(89.5, -0.8, 0, runtime);

                    lowerWobble();
                    sleep(1000);
                    strafeByDeadWheels(5, 0.3, false, 0, runtime);

                    raiseWobble();
                    sleep(1000);
                    moveByDeadWheels(94, 0.8, 0, runtime);
                    strafeByDeadWheels(30, 0.3, false, 0, runtime);
//                    turningPID(20,1,runtime);
                    dropWobble();
//                    turningPID(-20,1,runtime);
                    moveByDeadWheels(41, -0.8, 0, runtime);
//                    strafeByDeadWheels(5, 0.6, true, 0, runtime);
                    shoot(1, 1, 5);
                    moveByDeadWheels(5, 1, 0, runtime);
                    break;
                default:
                    // TODO: 9/29/20 add spline moveByDeadWheelsment there
                    System.out.println("a");
                    telemetry.addLine("a");
                    telemetry.update();
//                    strafeByDeadWheels(4, 0.2, false, 0, runtime);
                    moveByDeadWheels(63, 0.7, 0, runtime);
                    dropWobble();
                    strafeByDeadWheels(39, 0.5, true, 0, runtime);
                    moveByDeadWheels(43, -0.7, 0, runtime);
                    lowerWobble();
                    sleep(1000);
                    strafeByDeadWheels(3, 0.3, false, 0, runtime);
                    raiseWobble();
                    sleep(1000);
                    moveByDeadWheels(57, 0.7, 0, runtime);
                    strafeByDeadWheels(34, 0.5, false, 0, runtime);
                    dropWobble();
                    strafeByDeadWheels(8, 0.5, true, 0, runtime);
                    moveByDeadWheels(8, -0.5, 0, runtime);
//                    shoot(1, 1, 4);
//                    strafeByDeadWheels(30, 0.5, true, 0, runtime);
                    //  powerShot(runtime);
                    //   moveByDeadWheels(8, 0.5, 0, runtime);
//                    moveByDeadWheels(63, 0, 1, 0, runtime);
//                    moveByDeadWheels(20, -0.8, 0, runtime);
//                    strafeByDeadWheels(4, 0.5, true, 0, runtime);
                    moveByDeadWheels(3, 0.8, 0, runtime);
                    shoot(.8, 1, 4);
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

    public void powerShot(ElapsedTime runtime) throws HeartBeatException {

        turnOffEncoders();
        strafeByDeadWheels(9, 0.5, true, 0, runtime);
        shoot(0.8, 1, 0.25);
        strafeByDeadWheels(4, 0.5, true, 0, runtime);
        shoot(0.8, 1, 0.35);
        strafeByDeadWheels(4, 0.5, true, 0, runtime);
        shoot(0.8, 1, 0.25);
        turnOnEncoders();
    }
}
