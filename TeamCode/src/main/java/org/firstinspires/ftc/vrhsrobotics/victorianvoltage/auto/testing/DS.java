package org.firstinspires.ftc.vrhsrobotics.victorianvoltage.auto.testing;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.vrhsrobotics.victorianvoltage.auto.Auto;


@TeleOp(name = "dst")
//@Disabled
public class DS extends Auto {

    @Override
    public void runOpMode() throws InterruptedException {
       DcMotorEx shootR = (DcMotorEx) hardwareMap.dcMotor.get("shootR");
//        DcMotorEx shootL = (DcMotorEx) hardwareMap.dcMotor.get("shootL");
        DcMotorEx intake = (DcMotorEx) hardwareMap.dcMotor.get("intake");

        shootR.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        intake.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
//        shootL.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        double DEAD_WHEEL_TICKS_PER_REV = 4096;
        double DEAD_WHEEL_TO_TICKS = DEAD_WHEEL_TICKS_PER_REV / (Math.PI * 2);
        waitForStart();


        while (opModeIsActive())
        {
            resetDeadWheels();
            telemetry.clear();
            telemetry.addData("forward right", shootR.getCurrentPosition() / DEAD_WHEEL_TO_TICKS);
//        telemetry.addData("forward Left",shootL.getCurrentPosition());
            telemetry.addData("horizontal", intake.getCurrentPosition() / DEAD_WHEEL_TO_TICKS);
            telemetry.update();
        }
    }
}
