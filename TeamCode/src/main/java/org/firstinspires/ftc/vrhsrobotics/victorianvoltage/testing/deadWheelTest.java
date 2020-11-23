package org.firstinspires.ftc.vrhsrobotics.victorianvoltage.testing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.vrhsrobotics.victorianvoltage.auto.Auto;

@Autonomous(name = "dead wheels test")
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
