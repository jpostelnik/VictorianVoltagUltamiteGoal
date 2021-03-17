package org.firstinspires.ftc.vrhsrobotics.victorianvoltage.testing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.vrhsrobotics.victorianvoltage.auto.Auto;

@Autonomous(name = "diagonal strafe test")
@Disabled

public class digonalStrafeTest extends Auto {
    @Override
    public void runOpMode() throws InterruptedException {
        initialize();
        ElapsedTime runtime = new ElapsedTime();
        waitForStart();
        runtime.reset();
        diagonalStrafeOld(5, 5, 0.5, 0, runtime);
        diagonalStrafeOld(10, 8, 0.5, 0, runtime);
        diagonalStrafeOld(5, 8, 0.5, 0, runtime);

    }
}
