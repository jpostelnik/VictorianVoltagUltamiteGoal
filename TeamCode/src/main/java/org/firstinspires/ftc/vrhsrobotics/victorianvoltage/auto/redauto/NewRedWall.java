package org.firstinspires.ftc.vrhsrobotics.victorianvoltage.auto.redauto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.ejml.simple.SimpleMatrix;
import org.firstinspires.ftc.vrhsrobotics.victorianvoltage.auto.Auto;
import org.firstinspires.ftc.vrhsrobotics.victorianvoltage.auto.exceptions.HeartBeatException;
import org.firstinspires.ftc.vrhsrobotics.victorianvoltage.auto.vision.SkystoneDeterminationPipeline;


@Autonomous(name = "new red Wall")
public class NewRedWall extends Auto {

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

            location = SkystoneDeterminationPipeline.RingPosition.NONE;
            switch (location) {
                case ONE:
                    System.out.println("b");
                    telemetry.addLine("b");
                    telemetry.update();
                    move(new SimpleMatrix(new double[][]{{4}, {0}}), 0.3, runtime);
                    move(new SimpleMatrix(new double[][]{{0}, {90}}), 0.8, runtime);
                    move(new SimpleMatrix(new double[][]{{-6}, {0}}), 0.3, runtime);
                    dropWobble();
                    move(new SimpleMatrix(new double[][]{{-20}, {0}}), 0.7, runtime);
                    move(new SimpleMatrix(new double[][]{{0}, {-65}}), 0.8, runtime);
//
                    lowerWobble();
                    sleep(1000);
                    move(new SimpleMatrix(new double[][]{{4}, {0}}), 0.3, runtime);
                    raiseWobble();
                    sleep(1000);
                    move(new SimpleMatrix(new double[][]{{0}, {71}}), 0.8, runtime);
                    move(new SimpleMatrix(new double[][]{{10}, {0}}), 0.6, runtime);
                    dropWobble();
                    move(new SimpleMatrix(new double[][]{{0}, {-15}}), 0.8, runtime);
//                    //  strafeByDeadWheels(5, 0.6, true, 0, runtime);
//                    // shoot(0.8, 1);
                    break;
                case FOUR:
                    System.out.println("c");
                    telemetry.addLine("c");
                    telemetry.update();
                    move(new SimpleMatrix(new double[][]{{4}, {0}}), 0.3, runtime);
                    move(new SimpleMatrix(new double[][]{{0}, {110}}), 0.8, runtime);
                    move(new SimpleMatrix(new double[][]{{-5}, {0}}), 0.3, runtime);
                    dropWobble();
                    move(new SimpleMatrix(new double[][]{{-42.5}, {0}}), 0.3, runtime);
                    move(new SimpleMatrix(new double[][]{{0}, {-87}}), 0.8, runtime);
                    lowerWobble();
                    sleep(1000);
                    move(new SimpleMatrix(new double[][]{{4}, {0}}), 0.3, runtime);
                    raiseWobble();
                    sleep(1000);
                    move(new SimpleMatrix(new double[][]{{0}, {100}}), 0.8, runtime);
                    move(new SimpleMatrix(new double[][]{{30}, {0}}), 0.3, runtime);
                    dropWobble();
                    move(new SimpleMatrix(new double[][]{{0}, {-30}}), 0.8, runtime);
//                    // strafeByDeadWheels(5, 0.6, true, 0, runtime);
//                    //shoot(0.8, 1);
                    break;

                default:
                    // TODO: 9/29/20 add spline moveByDeadWheelsment there
                    System.out.println("a");
                    telemetry.addLine("a");
                    telemetry.update();
//                     strafeByDeadWheels(4, 0.2, false, 0, runtime);
                    move(new SimpleMatrix(new double[][]{{0}, {76}}), 0.7, runtime);
                    dropWobble();
                    move(new SimpleMatrix(new double[][]{{-39}, {0}}), 0.5, runtime);
                    move(new SimpleMatrix(new double[][]{{0}, {-44}}), 0.7, runtime);
                    lowerWobble();
                    sleep(1000);
                    move(new SimpleMatrix(new double[][]{{3}, {0}}), 0.3, runtime);
                    raiseWobble();
                    sleep(1000);
                    move(new SimpleMatrix(new double[][]{{0}, {57}}), 0.7, runtime);
                    move(new SimpleMatrix(new double[][]{{34}, {0}}), 0.5, runtime);
                    dropWobble();
                    move(new SimpleMatrix(new double[][]{{-8}, {0}}), 0.5, runtime);
                    move(new SimpleMatrix(new double[][]{{0}, {-8}}), 0.5, runtime);
                    move(new SimpleMatrix(new double[][]{{0}, {20}}), 0.8, runtime);
                    move(new SimpleMatrix(new double[][]{{-4}, {0}}), 0.5, runtime);
                    move(new SimpleMatrix(new double[][]{{}, {-3}}), 0.8, runtime);
                    shoot(.8, 1, 4);
                    break;
                //todo: will be what is in 0. So might removeByDeadWheels it.
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
