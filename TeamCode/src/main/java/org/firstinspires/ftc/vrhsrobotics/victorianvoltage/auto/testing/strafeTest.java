package org.firstinspires.ftc.vrhsrobotics.victorianvoltage.auto.testing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.vrhsrobotics.victorianvoltage.auto.Auto;
import org.firstinspires.ftc.vrhsrobotics.victorianvoltage.auto.exceptions.HeartBeatException;

@Autonomous(name = "Strafe")
@Disabled
public class strafeTest extends Auto {
    @Override
    public void runOpMode()  {
        ElapsedTime runtime = new ElapsedTime();
        initialize();
        waitForStart();
        resetStartTime();
        try {
            strafeByDeadWheels(12,0.5,false,0,runtime);
        } catch (HeartBeatException e) {
        }
//        strafe(12,0.5,false,0,runtime);
    }
}
