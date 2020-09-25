package auto;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

import auto.math.basicMathCalls;
import auto.math.controltheory.KalminFilter;
import auto.math.controltheory.PIDController;


/**
 * parent class for auto stuff
 */
public abstract class Auto extends LinearOpMode {

    private final double TICKS_PER_REV = 560;
    private final double DRIVETRAIN_WHEEL_DIAMTER = 4;
    private final double DRIVETRAIN_GEAR_RATIO = 1;
    private final double LINEAR_TO_TICKS = TICKS_PER_REV / (Math.PI * DRIVETRAIN_WHEEL_DIAMTER * DRIVETRAIN_GEAR_RATIO);
    private final double radius = 6.5;
    private final double STRAFE_COEFFICIENT = 1.20;
    private final double DEAD_WHEEL_DIAMTER = 2;
    private final double DEAD_WHEEL_TICKS_PER_REV = 4096;
    private final double DEAD_WHEEL_TO_TICKS = DEAD_WHEEL_TICKS_PER_REV / (Math.PI * DEAD_WHEEL_DIAMTER);

    private DcMotorEx rightFront, leftFront, rightRear, leftRear, intakeLeft, intakeRight, liftMotor, capMotor;
    private Servo hookL, clawL, gripL, towerBL, hookR, clawR, gripR, towerBR,towerTR,towerTL,guideL,guideR;
    private DigitalChannel liftTouch;

    private BNO055IMU imu;
    private ElapsedTime runtime = new ElapsedTime();

    private DcMotorEx[] driveTrain;

    private PIDController pid = new PIDController(runtime, 0, 0, 0);

    /**
     * checks if opmode is still running
     *
     * @throws InterruptedException
     */
    private void heartbeat() throws InterruptedException {
        if (!opModeIsActive()) {
            throw new InterruptedException();
        }
    }

    protected void restRuntime() {
        runtime.reset();
    }

    /**
     * inits motors
     */
    private void initMotors() {
        leftFront = (DcMotorEx) hardwareMap.dcMotor.get("leftFront");
        rightFront = (DcMotorEx) hardwareMap.dcMotor.get("rightFront");
        rightRear = (DcMotorEx) hardwareMap.dcMotor.get("rightRear");
        leftRear = (DcMotorEx) hardwareMap.dcMotor.get("leftRear");

        intakeLeft = (DcMotorEx) hardwareMap.dcMotor.get("intakeLeft");
        intakeRight = (DcMotorEx) hardwareMap.dcMotor.get("intakeRight");

        capMotor = (DcMotorEx) hardwareMap.dcMotor.get("capMotor");
        liftMotor = (DcMotorEx) hardwareMap.dcMotor.get("liftMotor");

        driveTrain = new DcMotorEx[]{rightFront, leftFront, rightRear, leftRear};

        for (DcMotorEx motor : driveTrain) {
            motor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
        leftFront.setDirection(DcMotor.Direction.FORWARD);
        leftRear.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        rightRear.setDirection(DcMotor.Direction.REVERSE);
    }

    /**
     * inits servos
     */
    private void initServos() {
        hookL = hardwareMap.servo.get("hookL");
        hookR = hardwareMap.servo.get("hookR");
        clawL = hardwareMap.servo.get("clawL");
        clawR = hardwareMap.servo.get("clawR");
        gripL = hardwareMap.servo.get("gripL");
        gripR = hardwareMap.servo.get("gripR");
        towerBL = hardwareMap.servo.get("towerBL");
        towerBR = hardwareMap.servo.get("towerBR");
        towerTL = hardwareMap.servo.get("towerTL");
        towerTR = hardwareMap.servo.get("towerTR");
        guideL = hardwareMap.servo.get("guideL");
        guideR = hardwareMap.servo.get("guideR");


        hookL.setPosition(1);
        hookR.setPosition(0);

        gripL.setPosition(0);
        gripR.setPosition(1);

        towerBL.setPosition(1);
        towerBR.setPosition(0);

        towerTR.setPosition(1);
        towerTL.setPosition(0);

        clawL.setPosition(0);
        clawR.setPosition(1);

        guideL.setPosition(0);
        guideL.setPosition(1);



    }

    /**
     * use for init the gyro
     */
    private void initGyro() {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
    }

    /**
     * this initialize every motor, servo, and sensor
     */
    protected void initialize() {
        initMotors();
        initServos();
        initGyro();
    }

    /**
     * checks if motors are busy
     *
     * @param ticks
     * @param startingPosition
     * @return
     */
    private boolean motorsBusy(int ticks, double startingPosition) {
        return Math.abs(leftRear.getCurrentPosition() - startingPosition) < ticks && Math.abs(rightRear.getCurrentPosition() - startingPosition) < ticks && Math.abs(leftFront.getCurrentPosition() - startingPosition) < ticks && Math.abs(rightFront.getCurrentPosition() - startingPosition) < ticks;
    }

    /**
     * stops the robot
     */
    private void halt() {
        for (DcMotorEx motor : driveTrain) {
            motor.setPower(0);
        }
    }

    /**
     * @param distance distance in inches that the robot move
     * @param power    it is how fast the robot moves
     * @param heading  the target heading for the front of the robot
     * @throws InterruptedException if op mod is not running then it stops
     */
    public void move(double distance, double power, int heading) throws InterruptedException {
        resetMotors();
        double currentPosition = leftRear.getCurrentPosition();
        int ticks = basicMathCalls.getTicksMotor(distance, "linear");
        telemetry.addData("starting position", currentPosition);
        telemetry.update();
        for (DcMotorEx motor : driveTrain) {
            motor.setPower(power);
        }
        while (motorsBusy(ticks, currentPosition)) ;
        {
            correction(heading, "strait", power, 1);
            heartbeat();

        }
        telemetry.addData("ending position", leftRear.getCurrentPosition());
        telemetry.update();

        halt();
    }

    /**
     *
     * @param time
     * @param power
     * @param runtime
     * @throws InterruptedException
     */
    protected void moveByTime(double time, double power, ElapsedTime runtime) throws InterruptedException {
        resetMotors();
        double statingPosition = leftRear.getCurrentPosition();

        double endTime = runtime.seconds() + time;

        for (DcMotorEx motor : driveTrain) {
            motor.setPower(power);
        }
        while (runtime.seconds()<endTime)
        {
            heartbeat();
        }
        halt();

        double endingPositon = leftRear.getCurrentPosition();

        telemetry.addData("change in positions in ticks", (endingPositon + statingPosition));
        telemetry.update();
    }

    /**
     * this rests the encoder positions to 0
     */
    private void resetMotors() {

        for (DcMotorEx motor : driveTrain) {
            motor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        }
    }

    /**
     * moves the robot in a lateral direction
     *
     * @param distance where i want to go
     * @param power    the speed
     * @param right    direction
     * @throws InterruptedException
     */
    protected void strafe(double distance, double power, boolean right, int heading) throws InterruptedException {
        resetMotors();

        double currentPosition = leftRear.getCurrentPosition();

        int ticks = basicMathCalls.getTicksMotor(distance, "strafe");

        if (right) {
            leftRear.setPower(1 * power);
            leftFront.setPower(-1 * power);
            rightRear.setPower(-1 * power);
            rightFront.setPower(1 * power);
        } else {
            leftRear.setPower(-1 * power);
            leftFront.setPower(1 * power);
            rightRear.setPower(1 * power);
            rightFront.setPower(-1 * power);

        }

        while (motorsBusy(ticks, currentPosition)) ;
        {
            heartbeat();

        }
        halt();

    }

    /**
     * used for turning just by ticks
     *
     * @param power     sets the speed
     * @param degrees   degree of a circle
     * @param direction either "right" or "left"
     * @throws InterruptedException throws if heartbeat throws
     */
    protected void turnTicks(double power, double degrees, String direction) throws InterruptedException {
        resetMotors();

        double currentPosition = leftRear.getCurrentPosition();
        double radians = degrees * Math.PI / 90;
        int ticks = (int) (radius * radians * LINEAR_TO_TICKS);

        if (direction.contains("left")) {
            rightFront.setPower(power * -1);
            rightRear.setPower(-1 * power);
            leftRear.setPower(power);
            leftFront.setPower(power);
        } else {
            rightFront.setPower(power);
            rightRear.setPower(power);
            leftRear.setPower(-1 * power);
            leftFront.setPower(-1 * power);
        }

        while (motorsBusy(ticks, currentPosition)) {
            heartbeat();
        }
        halt();
    }

    /**
     * this turns the robot using a pid
     *
     * @param target   degrees that i want
     * @param powerMax the max power od the turn
     * @param runtime  the current time used during the pid
     * @throws InterruptedException throws if heartbeat throws
     */
    protected void turningPID(double target, double powerMax, ElapsedTime runtime) throws InterruptedException {
        PIDController pid = new PIDController(runtime, 0.9728499206, 0, 0);
        resetMotors();
        pid.reset(runtime);
        double error;
        do {
            double current = getCurrentAngle();
            error = KalminFilter.getError(current, target);

            double correction = pid.correction(error, runtime);

            leftRear.setPower(Range.clip(-correction, -powerMax, powerMax));
            leftFront.setPower(Range.clip(-correction, -powerMax, powerMax));
            rightRear.setPower(Range.clip(correction, -powerMax, powerMax));
            rightFront.setPower(Range.clip(correction, -powerMax, powerMax));

            heartbeat();


        } while (Math.abs(error) > 3.5);

        halt();
    }


    /**
     * gets the current angle
     *
     * @return returns the angle in degrees
     */
    protected double getCurrentAngle() {
        return imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;

    }

    /**
     * 0 deggres is the middle of the left side if you look at it from the back and the right if from the front
     * if both positive the robot will move forward and to the left
     * if x is negative and y is positive then the robot will move left and back
     * if x is poositive and y is negative then the robot will move back and right
     * if both are negative it moves right and forward
     *
     * @param xDistance
     * @param yDistance
     * @param power
     * @throws InterruptedException
     */
    protected void diagonalStrafe(double xDistance, double yDistance, double power, int heading, ElapsedTime runtime) throws InterruptedException {
        resetDeadWheels();
        resetMotors();
        pid.reset(runtime);
        double degrees = basicMathCalls.getAngleDegrees(xDistance, yDistance);
        double secondaryPower = basicMathCalls.getSecondPower(degrees, power);
        double ticksX = xDistance * DEAD_WHEEL_TO_TICKS;
        double ticksY = yDistance * DEAD_WHEEL_TO_TICKS;

        double startingX = liftMotor.getCurrentPosition();
        double startingY = capMotor.getCurrentPosition();

        if (degrees > 90 || degrees < -90) {
            power *= -1;
        }
        boolean inv = false;
        if (degrees < 90 && degrees > -90) {
            inv = false;
            telemetry.addLine("right");
            telemetry.update();
            leftFront.setPower(secondaryPower);
            leftRear.setPower(power);
            rightFront.setPower(power);
            rightRear.setPower(secondaryPower);
        } else {
            inv = true;
            telemetry.addLine("left");
            telemetry.update();
            leftFront.setPower(power);
            leftRear.setPower(secondaryPower);
            rightFront.setPower(secondaryPower);
            rightRear.setPower(power);
        }

        while (deadWheelsBusy(ticksY, ticksX, startingY, startingX)) {
            correction(heading, power, secondaryPower, 1, inv);
            heartbeat();
        }

        halt();

    }

    /**
     * checks if the deadwheels have made it to the required ticks
     *
     * @param ticksY    total y distance that the robot needs to travel
     * @param ticksX    total x distance that the robot needs to travel
     * @param startingY what the deadwheels were when they started in why postions
     * @param startingX what the deadwheels were when they started in x postions
     * @return returns true if the it has not reached the positions, false if it has. checks if to continue.
     */
    protected boolean deadWheelsBusy(double ticksY, double ticksX, double startingY, double startingX) {
        return (Math.abs((liftMotor.getCurrentPosition() - startingX)) < Math.abs(ticksY) && Math.abs(capMotor.getCurrentPosition() - startingY) < Math.abs(ticksX));
    }

    /**
     * @param targetHeading
     * @param type
     * @param power
     * @param max
     */
    private void correction(double targetHeading, String type, double power, double max) {
        PIDController pid = new PIDController(runtime, 1.3, 0, 0);

        double target = targetHeading;
        double current = getCurrentAngle();

        double error = KalminFilter.getError(current, target);

        double correction = pid.correction(error, runtime);

        double rightP = (Range.clip(power + correction, -max, max));
        double leftP = Range.clip(power - correction, -max, max);

        leftRear.setPower(leftP);
        leftFront.setPower(leftP);
        rightRear.setPower(rightP);
        rightFront.setPower(rightP);

    }

    /**
     *
     */
    private void resetDeadWheels() {
        liftMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        capMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        capMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
    }

    /**
     * @param xCords
     * @param yCords
     * @param power
     * @param heading
     * @throws InterruptedException
     */
    protected void spline(double[] xCords, double[] yCords, double power, int heading, ElapsedTime runtime) throws InterruptedException {
        diagonalStrafe(xCords[0], yCords[0], power, heading, runtime);
        for (int i = 1; i < xCords.length; i++) {
            diagonalStrafe(xCords[i] - xCords[i - 1], yCords[i] - yCords[i - 1], power, heading, runtime);
        }
    }

    /**
     * `
     *
     * @param targetHeading
     * @param power
     * @param secondPower
     * @param max
     * @param inv
     */
    private void correction(double targetHeading, double power, double secondPower, double max, boolean inv) {

        double target = targetHeading;
        double current = getCurrentAngle();

        double error = KalminFilter.getError(current, target);

        double correction = pid.correction(error, runtime);

        double rightPowerMain = (Range.clip(power + correction, -max, max));
        double leftPowerMain = Range.clip(power - correction, -max, max);
        double leftPowerSecondary = Range.clip(secondPower - correction, -max, max);
        double rightSecondaryPower = Range.clip(secondPower + correction, -max, max);

        if (inv) {
            leftRear.setPower(leftPowerSecondary);
            leftFront.setPower(leftPowerMain);
            rightRear.setPower(rightPowerMain);
            rightFront.setPower(rightSecondaryPower);
        } else {
            leftRear.setPower(leftPowerMain);
            leftFront.setPower(leftPowerSecondary);
            rightRear.setPower(rightSecondaryPower);
            rightFront.setPower(rightPowerMain);
        }

    }

    public void shoot(double power)throws InterruptedException{
        while(true)
        {
            intakeLeft.setPower(power);
            heartbeat();
        }

    }
}



