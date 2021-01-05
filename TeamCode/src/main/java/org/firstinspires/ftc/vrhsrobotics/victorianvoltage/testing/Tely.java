package org.firstinspires.ftc.vrhsrobotics.victorianvoltage.testing;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.vrhsrobotics.victorianvoltage.auto.Auto;

@TeleOp(name = "kms")
public class Tely extends Auto {

    private final double TICKS_PER_REV = 1120;
    private final double DRIVETRAIN_WHEEL_DIAMTER = 4;
    private final double DRIVETRAIN_GEAR_RATIO = 5 / 6;
    private final double LINEAR_TO_TICKS = TICKS_PER_REV / (Math.PI * DRIVETRAIN_WHEEL_DIAMTER * DRIVETRAIN_GEAR_RATIO);

    private DcMotor rightFront, leftFront, rightRear, leftRear, shootR, shootL;
    private Servo elevator;

    ElapsedTime runTime = new ElapsedTime();
    double servoPostion = 0.45;

    @Override
    public void runOpMode() {
        leftFront = hardwareMap.dcMotor.get("leftFront");
        rightFront = hardwareMap.dcMotor.get("rightFront");
        rightRear = hardwareMap.dcMotor.get("rightRear");
        leftRear = hardwareMap.dcMotor.get("leftRear");

        leftFront.setDirection(DcMotor.Direction.FORWARD);
        leftRear.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        rightRear.setDirection(DcMotor.Direction.REVERSE);

        shootL = hardwareMap.dcMotor.get("shootL");
        shootR = hardwareMap.dcMotor.get("shootR");

        shootL.setDirection(DcMotor.Direction.REVERSE);
        shootR.setDirection(DcMotor.Direction.REVERSE);


        //servos
        elevator = hardwareMap.servo.get("elevator");


        telemetry.addLine("robot ready");
        telemetry.update();

        waitForStart();
        boolean isTop = false;
        runTime.reset();
        boolean shooterOn = false;
        while (opModeIsActive()) {
            double drive = -gamepad1.left_stick_y;
            double turn = -gamepad1.right_stick_y;

            double leftPower = Range.clip(drive - turn, -1, 1);
            double rightPower = Range.clip(drive + turn, -1, 1);


            leftRear.setPower(leftPower);
            leftFront.setPower(leftPower);
            rightRear.setPower(rightPower);
            rightFront.setPower(rightPower);

//            if (gamepad2.x) {
//                if (!isTop || elevator.getPosition() < 0.5) {
//                    isTop = true;
//                    elevator.setPosition(1.0);
//                } else {
//                    elevator.setPosition(0.0);
//                    isTop = false;
//
//                }
//            }
            if (gamepad2.dpad_up) {
                elevator.setPosition(1.0);
            }
            if(gamepad2.dpad_down){
                elevator.setPosition(0);

            }
            if(gamepad2.x)
            {
               double a =  elevator.getPosition();
               elevator.setPosition(a+0.05);
            }
            if(gamepad2.b)
            {
                double a =  elevator.getPosition();
                elevator.setPosition(a-0.05);
            }
            if (gamepad2.a) {
                if (!shooterOn) {
                    shootL.setPower(1);
                    shootR.setPower(1);
                    shooterOn = true;
                } else {
                    shootL.setPower(0);
                    shootR.setPower(0);
                    shooterOn = false;
                }
            }
            telemetry.addData("Runtime", runTime.seconds());
            telemetry.update();
        }

    }
}
