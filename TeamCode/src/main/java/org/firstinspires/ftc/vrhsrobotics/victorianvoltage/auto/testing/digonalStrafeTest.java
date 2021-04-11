package org.firstinspires.ftc.vrhsrobotics.victorianvoltage.auto.testing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.vrhsrobotics.victorianvoltage.auto.Auto;
import org.firstinspires.ftc.vrhsrobotics.victorianvoltage.auto.exceptions.HeartBeatException;

@Autonomous(name = "diagonal strafe test")
@Disabled

public class digonalStrafeTest extends Auto {
    @Override
    public void runOpMode()  {
        initialize();
        ElapsedTime runtime = new ElapsedTime();
        waitForStart();
        runtime.reset();

        try {
            diagonalStrafeOld(5, 5, 0.5, 0, runtime);
            diagonalStrafeOld(10, 8, 0.5, 0, runtime);
            diagonalStrafeOld(5, 8, 0.5, 0, runtime);
        } catch (HeartBeatException e) {
            e.printStackTrace();
        }

    }
}
