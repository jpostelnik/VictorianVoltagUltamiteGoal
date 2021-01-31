package org.firstinspires.ftc.vrhsrobotics.victorianvoltage.testing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.vrhsrobotics.victorianvoltage.auto.Auto;

@Autonomous(name = "Strafe")
public class strafeTest extends Auto {
    @Override
    public void runOpMode() throws InterruptedException {
        ElapsedTime runtime = new ElapsedTime();
        initialize();
        waitForStart();
        resetStartTime();
        strafeByDeadWheels(12,0.5,false,0,runtime);
        strafe(12,0.5,false,0,runtime);
    }
}
