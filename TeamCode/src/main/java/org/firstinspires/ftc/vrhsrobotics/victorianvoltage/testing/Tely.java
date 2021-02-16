package org.firstinspires.ftc.vrhsrobotics.victorianvoltage.testing;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.vrhsrobotics.victorianvoltage.auto.helper.PositionTracker;

@TeleOp(name = "telly")
public class Tely extends OpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotorEx rightFront, leftFront, rightRear, leftRear, shootR, shootL, intake, capMotor;
    private Servo arm, hook, liftPlateLeft, liftPlateRight;
    private boolean precision, direction, precisionChanged, directionChanged;
    private boolean useOneGamepad;
    private boolean positionChanged = false;
    private boolean closed = false, down = false, shooting = false, lowered = false, grounded = false;
    private boolean deployerPositionChanged = false;
    private boolean deployerClosed = false;
    private double baseParallelRightPosition;
    private double baseParallelLeftPosition;
    private double basePerpendicularPosition;
    private final int deadWheelTicks = 4096;
    private final double WHEEL_CIRCUMFERENCE_IN = Math.PI * 3.05; //circumference of parallel deadwheel
    private final double DEADWHEEL_INCHES_OVER_TICKS = WHEEL_CIRCUMFERENCE_IN / deadWheelTicks;
    private double time = -999;
    private PositionTracker positionTracker = new PositionTracker(0, 0, 0);

    @Override
    public void init() {
        //after driver hits init
        setUpDriveTrain();
        setUpIntake();
        setUpShoot();
        setUpServos();

        precision = false;
        direction = false;
        useOneGamepad = false;
        precisionChanged = false;
        directionChanged = false;
        shooting = false;

        closed = false;
        positionChanged = false;

        telemetry.addData("Status", "Initialized");

        baseParallelLeftPosition = shootL.getCurrentPosition();
        baseParallelRightPosition = shootR.getCurrentPosition();
        basePerpendicularPosition = intake.getCurrentPosition();
    }

    public void setUpDriveTrain() {
        leftFront = (DcMotorEx) hardwareMap.dcMotor.get("leftFront");
        rightFront = (DcMotorEx) hardwareMap.dcMotor.get("rightFront");
        rightRear = (DcMotorEx) hardwareMap.dcMotor.get("rightRear");
        leftRear = (DcMotorEx) hardwareMap.dcMotor.get("leftRear");


        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftRear.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        rightRear.setDirection(DcMotor.Direction.FORWARD);

        telemetry.addLine("drive train done");
        telemetry.update();

    }

    public void setUpIntake() {
        intake = (DcMotorEx) hardwareMap.dcMotor.get("intake");
        intake.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        telemetry.addLine("deadwheels done");
        telemetry.update();
    }

    public void setUpShoot() {
        shootR = (DcMotorEx) hardwareMap.dcMotor.get("shootR");
        shootL = (DcMotorEx) hardwareMap.dcMotor.get("shootL");
        shootR.setDirection(DcMotorSimple.Direction.REVERSE);
        shootL.setDirection(DcMotorSimple.Direction.REVERSE);
        shootR.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        shootL.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        shootR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        shootL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


    }

    public void setUpServos() {
        arm = hardwareMap.servo.get("arm");
        arm.setPosition(1);

        hook = hardwareMap.servo.get("hook");
        hook.setPosition(1);

        liftPlateLeft = hardwareMap.servo.get("liftPlateLeft");
        liftPlateLeft.setPosition(0);

        liftPlateRight = hardwareMap.servo.get("liftPlateRight");
        liftPlateRight.setPosition(0);
    }

    @Override
    //what runs in between robot being initialized and before it plays
    public void init_loop() {

    }

    //once driver hits play
    @Override
    public void start() {
        runtime.reset();
    }


    @Override
    public void loop() {
        telemetry.update();
        driveBot();
        intake();
        shoot();
        gripWobble();
        moveArm();
        lowerPlate();
        useEncoders();


    }

    public void useEncoders() {
        double parallelLeftTicks = (shootL.getCurrentPosition() - baseParallelLeftPosition);
        double parallelRightTicks = shootR.getCurrentPosition() - baseParallelRightPosition;
        double perpendicularTicks = intake.getCurrentPosition() - basePerpendicularPosition;

        positionTracker.updateTicks(parallelLeftTicks, parallelRightTicks, perpendicularTicks);

        if (Math.abs(gamepad1.left_stick_x) < Math.cos(Math.toRadians(5)) || Math.sqrt(Math.pow(gamepad1.left_stick_x, 2) + Math.pow(gamepad1.left_stick_y, 2)) < .1) {
            positionTracker.updateLocationAndPose(telemetry, "");
        } else {
            positionTracker.updateLocationAndPose(telemetry, "");
        }

        telemetry.addData("parallel left ticks", parallelLeftTicks);
        telemetry.addData("parallel right ticks", parallelRightTicks);
        telemetry.addData("perpendicular ticks", perpendicularTicks);
        telemetry.addData("Distance traveled", (parallelLeftTicks + parallelRightTicks) * DEADWHEEL_INCHES_OVER_TICKS / 2);
        telemetry.addData("X Position", positionTracker.getCurrentX());
        telemetry.addData("Y Position", positionTracker.getCurrentY());
        telemetry.update();
    }

    public void driveBot() {
        if ((gamepad1.a && !precisionChanged && !useOneGamepad) || (useOneGamepad && gamepad1.y)) {
            precision = !precision;
            precisionChanged = true;
        } else if ((!useOneGamepad && !gamepad1.a) || (useOneGamepad && !gamepad1.y)) {
            precisionChanged = false;
        }

//        if (gamepad1.x && !directionChanged && !useOneGamepad) {
//            direction = !direction;
//            directionChanged = true;
//        }
//        else if (!gamepad1.x && !useOneGamepad) {
//            directionChanged = false;
//        }

        double xMagnitude = Math.hypot(gamepad1.left_stick_x, gamepad1.left_stick_y);
        double xLinear = direction ? xMagnitude : -xMagnitude;

        double joystickAngle = Math.atan2(gamepad1.left_stick_y, gamepad1.left_stick_x) + 3 * Math.PI / 4;
        double xTurn = gamepad1.right_stick_x;


        double leftFrontPower = xLinear * Math.sin(joystickAngle) - xTurn;
        double rightFrontPower = xLinear * Math.cos(joystickAngle) + xTurn;
        double leftBackPower = xLinear * Math.cos(joystickAngle) - xTurn;
        double rightBackPower = xLinear * Math.sin(joystickAngle) + xTurn;

        double[] motorPowers = new double[]{leftFrontPower, rightFrontPower, leftBackPower, rightBackPower};
        motorPowers = convertMotorPowers(motorPowers, xLinear, xTurn);

        leftFront.setPower(precision ? 0.4 * motorPowers[0] : motorPowers[0]);
        rightFront.setPower(precision ? 0.4 * motorPowers[1] : motorPowers[1]);
        leftRear.setPower(precision ? 0.4 * motorPowers[2] : motorPowers[2]);
        rightRear.setPower(precision ? 0.4 * motorPowers[3] : motorPowers[3]);

//        telemetry.addData("Front Motors", "Left Front (%.2f), Right Front (%.2f)", leftFront.getPower(), rightFront.getPower());
//        telemetry.addData("joystickAngle", joystickAngle);
//        telemetry.addData("Rear Motors", "Left Rear (%.2f), Right Rear (%.2f)", leftBack.getPower(), rightBack.getPower());
    }

    public double[] convertMotorPowers(double[] motorPowers, double xLinear, double xTurn) {
        double maxPower = getMaxMagnitude(motorPowers);

        double conversion = Math.abs(Math.sqrt((Math.pow(xLinear, 2) + Math.pow(xTurn, 2))/*/2*/) / maxPower);

//        telemetry.addData("maxPower", maxPower);
//        telemetry.addData("conversion", conversion);
//        telemetry.update();

        for (int i = 0; i < motorPowers.length; i++) {
            motorPowers[i] *= conversion;
        }

        return motorPowers;
    }

    public double getMaxMagnitude(double[] arr) {
        double max = Math.abs(arr[0]);

        for (int i = 1; i < arr.length; i++) {
            if (Math.abs(arr[i]) > max) {
                max = arr[i];
            }
        }

        return max;
    }

    public void intake() {
        if (gamepad1.right_bumper) {
            intake.setPower(1);
        } else if (gamepad1.left_bumper) {
            intake.setPower(-1);
        } else {
            intake.setPower(0);
        }
    }


    public void gripWobble() {
        if (gamepad1.dpad_down) {
            hook.setPosition(closed ? 1 : 0.1);
            closed = !closed;
        } else if ((!useOneGamepad && !gamepad2.b) || (useOneGamepad && !gamepad1.b)) {
            positionChanged = false;
        }
    }

    public void moveArm() {
        if (gamepad1.dpad_up) {
            arm.setPosition(lowered ? 1 : 0.1);
            lowered = !lowered;
        } else if ((!useOneGamepad && !gamepad2.b) || (useOneGamepad && !gamepad1.b)) {
            positionChanged = false;
        }
    }

    public void
    shoot() {
        if (gamepad1.left_trigger>0.8) {
            shootR.setPower(0.8);
            shootL.setPower(0.8);

        }
        else if(gamepad1.right_trigger>0.8){
            shootR.setPower(-0.8);
            shootL.setPower(-0.8);
        }else{
            shootR.setPower(0);
            shootL.setPower(0);
        }
    }

    public void lowerPlate() {
        if (gamepad1.y) {
            liftPlateLeft.setPosition(grounded ? 1 : 0);
            liftPlateRight.setPosition(grounded ? 0 : 1);

            grounded = !grounded;
            positionChanged = true;
        } else if ((!useOneGamepad && !gamepad2.b) || (useOneGamepad && !gamepad1.b)) {
            positionChanged = false;
        }
    }


    public void useOneGamepad() {
        if ((gamepad1.right_bumper && gamepad1.left_bumper && gamepad1.left_trigger == 1 && gamepad1.right_trigger == 1) || (gamepad2.right_bumper && gamepad2.left_bumper && gamepad2.left_trigger == 1 && gamepad2.right_trigger == 1)) {
            useOneGamepad = !useOneGamepad;
        }
    }


    @Override
    public void stop() {

    }
}