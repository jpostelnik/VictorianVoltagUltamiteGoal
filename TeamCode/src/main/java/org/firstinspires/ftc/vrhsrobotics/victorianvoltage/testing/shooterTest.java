package org.firstinspires.ftc.vrhsrobotics.victorianvoltage.testing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.vrhsrobotics.victorianvoltage.auto.Auto;
@Autonomous(name = "shooter Test")
public class shooterTest extends Auto {
    private ElapsedTime runtime = new ElapsedTime();
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addLine("starting to initialize");
        telemetry.update();
        initialize();
        waitForStart();
        runtime.reset();

        restRuntime();
        try {
            shoot(1, 1, 4);
        }catch (Exception e){
            e.printStackTrace();
        }

    }
}
