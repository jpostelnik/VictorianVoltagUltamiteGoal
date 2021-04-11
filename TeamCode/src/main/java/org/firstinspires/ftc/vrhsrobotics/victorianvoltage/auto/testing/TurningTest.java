package org.firstinspires.ftc.vrhsrobotics.victorianvoltage.auto.testing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.vrhsrobotics.victorianvoltage.auto.Auto;
import org.firstinspires.ftc.vrhsrobotics.victorianvoltage.auto.exceptions.HeartBeatException;

@Autonomous(name = "turning test")
public class TurningTest extends Auto {
    @Override
    public void runOpMode() {
        ElapsedTime runtime = new ElapsedTime();
        initialize();
        waitForStart();
        resetStartTime();
        try {
            turningPID(20, 1, runtime);
        } catch (HeartBeatException e) {
        }
    }
}
