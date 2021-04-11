package org.firstinspires.ftc.vrhsrobotics.victorianvoltage.auto.testing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.vrhsrobotics.victorianvoltage.auto.Auto;

@Autonomous(name = "dead wheels test")
@Disabled
public class deadWheelTest extends Auto {
    @Override
    public void runOpMode() throws InterruptedException {
        initMotors();
        waitForStart();
        resetDeadWheels();
        while (opModeIsActive())
        {
            try {
                printDeadWheel();

            }
            catch (Exception e)
            {

            }
        }

    }
}
