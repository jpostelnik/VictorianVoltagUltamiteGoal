package org.firstinspires.ftc.vrhsrobotics.victorianvoltage.auto;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.ejml.simple.SimpleMatrix;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.vrhsrobotics.victorianvoltage.auto.math.MotionProfiling.MotionProfiler;
import org.firstinspires.ftc.vrhsrobotics.victorianvoltage.auto.math.basicMathCalls;
import org.firstinspires.ftc.vrhsrobotics.victorianvoltage.auto.math.controltheory.KalmanFilter;
import org.firstinspires.ftc.vrhsrobotics.victorianvoltage.auto.math.controltheory.PIDController;
import org.firstinspires.ftc.vrhsrobotics.victorianvoltage.auto.math.controltheory.RobotKalmanFilter;
import org.firstinspires.ftc.vrhsrobotics.victorianvoltage.auto.math.spline.Bezier;
import org.firstinspires.ftc.vrhsrobotics.victorianvoltage.auto.math.spline.Spline;
import org.firstinspires.ftc.vrhsrobotics.victorianvoltage.auto.math.spline.Waypoint;
import org.firstinspires.ftc.vrhsrobotics.victorianvoltage.auto.vision.SkystoneDeterminationPipeline;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.io.File;

import static org.firstinspires.ftc.vrhsrobotics.victorianvoltage.auto.math.controltheory.RobotKalmanFilter.getError;


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


    private DcMotorEx rightFront, leftFront, rightRear, leftRear, shootR, shootL, intake, feeder;
    private Servo arm, hook, liftPlateLeft, liftPlateRight;

    private BNO055IMU imu;
    private ElapsedTime runtime = new ElapsedTime();

    private DcMotorEx[] driveTrain;

    private MotionProfiler motionProfiler;


    //pids
    private PIDController pid = new PIDController(runtime, 0.8, 0, 0.2);
    private PIDController pidTurning = new PIDController(runtime, 1, 0, 0);
    private PIDController pidStrafe = new PIDController(runtime, 0.08, 0, 0);
    private PIDController pidDiagonal = new PIDController(runtime, 0.01, 0, 0);
    private PIDController pidPositional = new PIDController(runtime, 0.7, 0.1, 0);

    protected WebcamName weCam;
    protected OpenCvCamera camera;
    protected SkystoneDeterminationPipeline pipeline;

    //KalmanFilter Filter
    // 10 updates per second
    double dt = 1.0 / 10;
    SimpleMatrix f = new SimpleMatrix(new double[][]{
            {1, 0, dt, 0},
            {0, 1, 0, dt},
            {0, 0, 1, 0},
            {0, 0, 0, 1}
    }
    );


    SimpleMatrix G = new SimpleMatrix(new double[][]{
            {0.5 * Math.pow(dt, 2)},
            {0.5 * Math.pow(dt, 2)},
            {dt},
            {dt}
    });

    double sigma_a = 1;
    SimpleMatrix Q = G.mult(G.transpose()).scale(sigma_a * sigma_a);

    SimpleMatrix H = SimpleMatrix.identity(f.numRows());

    double positionVar = 0.05;
    double velocityVar = 0.0001;

    SimpleMatrix R = new SimpleMatrix(new double[][]{
            {positionVar, 0, 0, 0},
            {0, positionVar, 0, 0},
            {0, 0, velocityVar, 0},
            {0, 0, 0, velocityVar}
    });

    KalmanFilter robotKalmanFilter = new KalmanFilter(f, G, R, Q, H);


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

    /**
     *
     */
    protected void restRuntime() {
        runtime.reset();
    }

    /**
     *
     */
    private void intSensors() {

    }

    /**
     * inits motors
     */
    public void initMotors() {
        leftFront = (DcMotorEx) hardwareMap.dcMotor.get("leftFront");
        rightFront = (DcMotorEx) hardwareMap.dcMotor.get("rightFront");
        rightRear = (DcMotorEx) hardwareMap.dcMotor.get("rightRear");
        leftRear = (DcMotorEx) hardwareMap.dcMotor.get("leftRear");

        driveTrain = new DcMotorEx[]{rightFront, leftFront, rightRear, leftRear};

        for (DcMotorEx motor : driveTrain) {
            motor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftRear.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        rightRear.setDirection(DcMotor.Direction.FORWARD);

        telemetry.addLine("drive train done");
        telemetry.update();

        shootR = (DcMotorEx) hardwareMap.dcMotor.get("shootR");
        shootL = (DcMotorEx) hardwareMap.dcMotor.get("shootL");
        intake = (DcMotorEx) hardwareMap.dcMotor.get("intake");
        feeder = (DcMotorEx) hardwareMap.dcMotor.get("feeder");

//        shootL.setDirection(DcMotor.Direction.REVERSE);
//        shootR.setDirection(DcMotor.Direction.REVERSE);
        feeder.setDirection(DcMotorSimple.Direction.REVERSE);

        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        feeder.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shootR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shootL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        if (shootR == null) {
            telemetry.addLine("shootR is null");
            telemetry.update();
        }


        if (intake == null) {
            telemetry.addLine("intake is null");
            telemetry.update();
        }
        shootR.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        intake.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        shootL.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        feeder.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);


        telemetry.addLine("deadwheels done");
        telemetry.update();

    }

    /**
     * inits servos
     */
    private void initServos() {
        arm = hardwareMap.servo.get("arm");
        arm.setPosition(0);

        hook = hardwareMap.servo.get("hook");
        hook.setPosition(1);

//        liftPlateLeft = hardwareMap.servo.get("liftPlateLeft");
//        liftPlateLeft.setPosition(1);
//
//        liftPlateRight = hardwareMap.servo.get("liftPlateRight");
//        liftPlateRight.setPosition(0);
    }

    /**
     *
     */
    protected void intCamera() {
        weCam = hardwareMap.get(WebcamName.class, "Webcam 1");
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(weCam, cameraMonitorViewId);
        pipeline = new SkystoneDeterminationPipeline();
        camera.setPipeline(pipeline);

        // We set the viewport policy to optimized view so the preview doesn't appear 90 deg
        // out when the RC activity is in portrait. We do our actual image processing assuming
        // landscape orientation, though.
//        camera.setViewportRenderingPolicy(OpenCvCamera.ViewportRenderingPolicy.OPTIMIZE_VIEW);

        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(320, 240, OpenCvCameraRotation.SIDEWAYS_RIGHT);
            }
        });
    }

    /**
     *
     */
    public void close() {
        camera.closeCameraDevice();
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
     *
     */
    protected void initialize() {
        initMotors();
        telemetry.addLine("motors int");
        telemetry.update();
        initServos();
        telemetry.addLine("servos int");
        telemetry.update();
        initGyro();
        telemetry.addLine("gyro int");
        telemetry.update();
//        initVision();
        intCamera();
        telemetry.addLine("vision int");
        telemetry.update();
        intSensors();
        telemetry.addLine("sensors int");


        telemetry.addLine("all init");
        telemetry.update();


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
    protected void halt() {
        for (DcMotorEx motor : driveTrain) {
            motor.setPower(0);
        }
    }


    /**
     * @param distance distance in inches that the robot move
     * @param power    it is how fast the robot moves
     * @param heading  the target heading for the front of the robot
     * @throws InterruptedException if op mod is not running then it stops
     *                              oof
     */
    @Deprecated
    public void moveOld(double distance, double power, int heading, ElapsedTime runtime) throws InterruptedException {
        resetMotors();
        pid.reset(runtime);
        double deadWheelStart = -1 * shootR.getCurrentPosition();
        ;
        double currentPosition = leftRear.getCurrentPosition();
        int ticks = basicMathCalls.getTicksMotor(distance, "linear");
//        int ticks = (int) (distance * LINEAR_TO_TICKS);
        telemetry.clear();
        telemetry.addData("starting distance", deadWheelStart / DEAD_WHEEL_TO_TICKS);
        telemetry.addData("ticks", ticks);
        telemetry.addData("starting position", currentPosition);
        telemetry.update();
        sleep(1000);
        for (DcMotorEx motor : driveTrain) {
            motor.setPower(power);
        }
        telemetry.addLine("motors set");
        telemetry.update();
        while (motorsBusy(ticks, currentPosition)) ;
        {
            telemetry.clear();
            telemetry.addData("ticks left", ticks - (leftRear.getCurrentPosition()));
            System.out.println("distance traveled " + (-1 * shootR.getCurrentPosition() - deadWheelStart) / DEAD_WHEEL_TO_TICKS);
            telemetry.addData("distance traveled", (shootR.getCurrentPosition() - deadWheelStart) / DEAD_WHEEL_TO_TICKS);
            correction(heading, "strait", power, 1);
            heartbeat();
            telemetry.update();

        }
        telemetry.addData("ending position", leftRear.getCurrentPosition());
        telemetry.addData("distance traveled", (-1 * shootR.getCurrentPosition() - deadWheelStart) / DEAD_WHEEL_TO_TICKS);
        telemetry.update();


        halt();
    }

    /**
     * @param distance
     * @param power
     * @param heading
     * @param runtime
     * @throws InterruptedException
     */
    public void moveByDeadWheels(double distance, double power, int heading, ElapsedTime runtime) throws InterruptedException {
        pid.reset(runtime);
        resetDeadWheels();
        double currentPosition = shootR.getCurrentPosition();
        int ticks = basicMathCalls.getDeadWheelTicks(distance);
        for (DcMotorEx motors : driveTrain) {
            motors.setPower(power);
        }
        while (Math.abs(shootR.getCurrentPosition() - currentPosition) < ticks) ;
        {
            correction(power, heading, "straight", false, 1);
            heartbeat();
        }
        halt();
    }


    public void move(double distanceX, double distanceY, double power, int heading, ElapsedTime runtime) throws InterruptedException {
        pid.reset(runtime);
        resetDeadWheels();
        robotKalmanFilter.setInitalPostion(new SimpleMatrix(new double[][]{
                        {0},
                        {0},
                        {0},
                        {0}
                }),
                new SimpleMatrix(new double[][]{
                        {0, 0, 0, 0},
                        {0, 0, 0, 0},
                        {0, 0, 1, 0},
                        {0, 0, 0, 5}
                }));
        resetDeadWheels();
        double currentPosition = shootR.getCurrentPosition();
        int ticksX = basicMathCalls.getDeadWheelTicks(distanceX);
        int ticksY = basicMathCalls.getDeadWheelTicks(distanceY);
        for (DcMotorEx motors : driveTrain) {
            motors.setPower(power);
        }
        double position = 0;
        SimpleMatrix targetVector = new SimpleMatrix(new double[][]{
                {ticksX},
                {ticksY}
        });
        double errorMagnitude = 1000000;
        while (errorMagnitude < 1) {
            double start = runtime.milliseconds();
            position = /*(*/shootR.getCurrentPosition() /*+ shootL.getCurrentPosition()) / 2*/;
            robotKalmanFilter.update(new SimpleMatrix(new double[][]{
                    {position},
                    {intake.getCurrentPosition()},
                    {shootR.getVelocity()},
                    {intake.getVelocity()}
            }));
            SimpleMatrix currentPositionVector = new SimpleMatrix(new double[][]{
                    {robotKalmanFilter.getXPosition()},
                    {robotKalmanFilter.getYPosition()}
            });

            SimpleMatrix errorVector = targetVector.minus(currentPositionVector);
            errorMagnitude = Math.sqrt(Math.pow(errorVector.get(0, 0), 2) + Math.pow(errorVector.get(0, 0), 2));
            double correction = pidPositional.correction(errorMagnitude, runtime);

            if (correction > 4000) {
                correction(power, heading, "straight", false, 1);

            } else {
                power *= correction / 4000;
                correction(power, heading, "straight", false, 1);

            }
            double end = runtime.milliseconds();
            double elapsedTime = end-start;
            telemetry.addData("elapsed time", elapsedTime);
            telemetry.update();
            heartbeat();
        }
        halt();
    }

    /**
     * @param distance
     * @param power
     * @param left
     * @param heading
     * @param runtime
     * @throws InterruptedException
     */
    public void strafeByDeadWheels(double distance, double power, boolean left, int heading, ElapsedTime runtime) throws InterruptedException {
        resetMotors();
        resetDeadWheels();
        pidStrafe.reset(runtime);
        double currentPosition = intake.getCurrentPosition();

        int ticks = basicMathCalls.getDeadWheelTicks(distance);
        String direction = "";
        if (left) {
            leftRear.setPower(1 * power);
            leftFront.setPower(-1 * power);
            rightRear.setPower(-1 * power);
            rightFront.setPower(1 * power);
            direction = "right";
        } else {
            leftRear.setPower(-1 * power);
            leftFront.setPower(1 * power);
            rightRear.setPower(1 * power);
            rightFront.setPower(-1 * power);
            direction = "left";
        }

        while (Math.abs(intake.getCurrentPosition() - currentPosition) < ticks) ;
        {
            correction(power, heading, "strafe", false, 1);
            heartbeat();
        }
        halt();
    }

    /**
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
        while (runtime.seconds() < endTime) {
            printTicks();
            heartbeat();
        }
        halt();
//
//        double endingPositon = leftRear.getCurrentPosition();
//
//        telemetry.addData("change in positions in ticks", (endingPositon + statingPosition));
//        telemetry.update();
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
     * @deprecated - use strafebyDeadwheels
     */
    @Deprecated
    protected void strafe(double distance, double power, boolean right, int heading, ElapsedTime runtime) throws InterruptedException {
        resetMotors();
        pidStrafe.reset(runtime);
        double currentPosition = leftRear.getCurrentPosition();

        int ticks = basicMathCalls.getTicksMotor(distance, "strafe");
        String direction = "";
        if (right) {
            direction = "right";
            leftRear.setPower(1 * power);
            leftFront.setPower(-1 * power);
            rightRear.setPower(-1 * power);
            rightFront.setPower(1 * power);
        } else {
            leftRear.setPower(-1 * power);
            leftFront.setPower(1 * power);
            rightRear.setPower(1 * power);
            rightFront.setPower(-1 * power);
            direction = "left";
        }

        while (motorsBusy(ticks, currentPosition)) ;
        {
            correction(heading, "strafe" + direction, power, 1);
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
    @Deprecated
    protected void turnTicks(double power, double degrees, String direction, ElapsedTime runtime) throws InterruptedException {
        resetMotors();
        pidStrafe.reset(runtime);
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
     * @param target   degrees that i want. now checks if the robot is going left or right so that we are not spinnign all the way around
     *                 this is to help with speed and effensiency
     * @param powerMax the max power od the turn
     * @param runtime  the current time used during the pid
     * @throws InterruptedException throws if heartbeat throws
     */
    protected void turningPID(double target, double powerMax, ElapsedTime runtime) throws InterruptedException {
        PIDController pid = new PIDController(runtime, 0.9728499206, 0, 0);
        resetMotors();
        telemetry.addData("angle", getCurrentAngle());
        calibrateBnO055();
        pid.reset(runtime);
        double error;

        do {
            double current = getCurrentAngle();
            error = getError(current, target);

            double correction = pid.correction(error, runtime);

            leftRear.setPower(Range.clip(correction, -powerMax, powerMax));
            leftFront.setPower(Range.clip(correction, -powerMax, powerMax));
            rightRear.setPower(Range.clip(-correction, -powerMax, powerMax));
            rightFront.setPower(Range.clip(-correction, -powerMax, powerMax));

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
        return imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYX, AngleUnit.DEGREES).firstAngle;

    }

    protected void printAngle() {
//        System.out.println(imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYX, AngleUnit.DEGREES).firstAngle);
//        System.out.println(imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).firstAngle);
//        System.out.println(imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XZX, AngleUnit.DEGREES).firstAngle);
//        System.out.println(imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XZX, AngleUnit.DEGREES).firstAngle);
//        System.out.println(imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.YXY, AngleUnit.DEGREES).firstAngle);
//        System.out.println(imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle);
//        System.out.println(imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.YXZ, AngleUnit.DEGREES).firstAngle);
//        System.out.println(imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.YZX, AngleUnit.DEGREES).firstAngle);
//        System.out.println(imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.YZY, AngleUnit.DEGREES).firstAngle);
//        System.out.println(imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle);
//        System.out.println(imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYZ, AngleUnit.DEGREES).firstAngle);
//        System.out.println(imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZXZ, AngleUnit.DEGREES).firstAngle);
//        System.out.println(imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZXY, AngleUnit.DEGREES).firstAngle);
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

        double startingX = 0;
        double startingY = 0;

        if (degrees > 90 || degrees < -90) {
            power *= -1;
        }
        boolean inv = false;
        if (degrees < 90 && degrees > -90) {
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
            correction(heading, power, secondaryPower, inv);
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
        return (Math.abs((shootR.getCurrentPosition() - startingX)) < Math.abs(ticksY) && Math.abs(intake.getCurrentPosition() - startingY) < Math.abs(ticksX));
    }

    /**
     * @param targetHeading
     * @param type
     * @param power
     * @param max
     */
    private void correction(double targetHeading, String type, double power, double max) {

        double target = targetHeading;
        double current = getCurrentAngle();

        telemetry.clear();
        double error = RobotKalmanFilter.getError(current, target);
        telemetry.addData("error: ", error);
        telemetry.update();

        if (type.equals("strait")) {
            double correction = pid.correction(error, runtime);
            telemetry.addData("correction: ", correction);
            telemetry.update();
            double rightP = (Range.clip(power + correction, -max, max));
            double leftP = Range.clip(power - correction, -max, max);

            leftRear.setPower(leftP);
            leftFront.setPower(leftP);
            rightRear.setPower(rightP);
            rightFront.setPower(rightP);
        } else if (type.equals("strafe")) {
            double correction = pidStrafe.correction(error, runtime);
            {
                if (type.contains("left")) {
                    leftFront.setPower(Range.clip(-power - correction, -1.0, 1.0));
                    rightFront.setPower(Range.clip(power + correction, -1.0, 1.0));
                    leftRear.setPower(Range.clip(power - correction, -1.0, 1.0));
                    leftRear.setPower(Range.clip(-power + correction, -1.0, 1.0));
                } else if (type.contains("right")) {
                    leftFront.setPower(Range.clip(power - correction, -1.0, 1.0));
                    rightFront.setPower(Range.clip(-power + correction, -1.0, 1.0));
                    leftRear.setPower(Range.clip(-power - correction, -1.0, 1.0));
                    rightRear.setPower(Range.clip(power + correction, -1.0, 1.0));
                }
            }
        }
    }

    /**
     *
     */
    public void resetDeadWheels() {
//        shootL.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        shootL.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        shootL.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        shootL.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
//        shootR.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        shootR.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        shootR.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        shootR.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
//        intake.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        intake.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        intake.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        intake.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
    }

    /**
     * @param xCords
     * @param yCords
     * @param power
     * @param heading
     * @throws InterruptedException
     */
    protected void spline(double[] xCords, double[] yCords, double power,
                          int heading, ElapsedTime runtime) throws InterruptedException {
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
     * @param inv
     */
    private void correction(double targetHeading, double power, double secondPower,
                            boolean inv) {

        double current = getCurrentAngle();
        double error = RobotKalmanFilter.getError(current, targetHeading);

        double correction = pidDiagonal.correction(error, runtime);

        double rightPowerMain = (Range.clip(power + correction, -1, 1));
        double leftPowerMain = Range.clip(power - correction, -1, 1);
        double leftPowerSecondary = Range.clip(secondPower - correction, -1, 1);
        double rightSecondaryPower = Range.clip(secondPower + correction, -1, 1);

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


    /**
     * for printing dead wheels while testing stuff
     *
     * @throws InterruptedException
     */
    public void printDeadWheel() throws InterruptedException {
        resetDeadWheels();
        while (true) {
            telemetry.clear();
            telemetry.addData("forward right", shootR.getCurrentPosition() / DEAD_WHEEL_TO_TICKS);
//        telemetry.addData("forward Left",shootL.getCurrentPosition());
            telemetry.addData("horizontal", intake.getCurrentPosition() / DEAD_WHEEL_TO_TICKS);
            telemetry.update();
            if (gamepad1.a)
                break;
            heartbeat();
        }

    }

    /**
     *
     */
    public void printTicks() {
        telemetry.clear();
        telemetry.addData("forward right", shootR.getCurrentPosition() / DEAD_WHEEL_TO_TICKS);
        telemetry.addData("forward Left", shootL.getCurrentPosition() / DEAD_WHEEL_TO_TICKS);
        telemetry.addData("horizontal", intake.getCurrentPosition() / DEAD_WHEEL_TO_TICKS);
        telemetry.addData("Right Front ticks", rightFront.getCurrentPosition() / LINEAR_TO_TICKS);
        telemetry.addData("Left Front ticks", leftFront.getCurrentPosition() / LINEAR_TO_TICKS);
        telemetry.addData("Right Rear ticks", rightRear.getCurrentPosition() / LINEAR_TO_TICKS);
        telemetry.addData("Left Rear ticks", leftRear.getCurrentPosition() / LINEAR_TO_TICKS);
        telemetry.addData("velocity", rightFront.getVelocity());


        telemetry.update();
    }

    /**
     * @param spline
     * @param arcLen
     * @param targetHeading
     * @param power
     * @throws InterruptedException
     */
    public void spline(Spline spline, double arcLen, double targetHeading, double power) throws
            InterruptedException {
        resetDeadWheels();
        resetMotors();
        double lastx;
        double lasty;
        lasty = intake.getCurrentPosition();
        double startPosition = lastx = shootR.getCurrentPosition();


        telemetry.addData("arc length", arcLen);
        double distance = 0;
        double t = 0;
        double maxTime = runtime.time() + 6;
        while (t < 1 /*&& (runtime.time() < maxTime)*/) {
            double xPosition = spline.getXLength(t);
            double yPosition = spline.getYLength(t);
            double theta = Math.atan2(xPosition - shootR.getCurrentPosition(), yPosition - intake.getCurrentPosition());
            theta = Math.toDegrees(theta);
            System.out.printf("angle: %2f", theta);
            System.out.printf("angle: %2f", theta);
            telemetry.clear();
            telemetry.addData("angle", theta);
            boolean inv = false;
            if (theta < 90 && theta > -90) {
                inv = true;
//                power *= -1;
            }
            double sp = Range.clip(basicMathCalls.getSecondPower(theta, power), -power, power);
            if (!inv) {
                leftFront.setPower(sp);
                leftRear.setPower(power);
                rightFront.setPower(power);
                rightRear.setPower(sp);
            } else {
                leftFront.setPower(power);
                leftRear.setPower(sp);
                rightFront.setPower(sp);
                rightRear.setPower(power);
            }

            correction(targetHeading, power, sp, inv);
//            correction(theta,"Spline",power,1);

            heartbeat();

            /*
              should now be correct just incase it goes backwoards on its Y axis or shootR
             */

            distance += Math.sqrt(Math.pow(Math.abs(-shootR.getCurrentPosition() - lastx), 2) + Math.pow(Math.abs(intake.getCurrentPosition() - lasty), 2));

            lastx = shootR.getCurrentPosition();
            lasty = intake.getCurrentPosition();
            t += 0.01;
//            t += Math.abs(distance / arcLen);
            telemetry.addData("t = ", t);
            telemetry.update();
        }
        halt();
    }

    /**
     *
     */
    private void calibrateBnO055() {
        BNO055IMU.CalibrationData calibrationData = imu.readCalibrationData();

        // Save the calibration data to a file. You can choose whatever file
        // name you wish here, but you'll want to indicate the same file name
        // when you initialize the IMU in an opmode in which it is used. If you
        // have more than one IMU on your robot, you'll of course want to use
        // different configuration file names for each.
        String filename = "AdafruitIMUCalibration.json";
        File file = AppUtil.getInstance().getSettingsFile(filename);
        ReadWriteFile.writeFile(file, calibrationData.serialize());
    }

    /**
     *
     */
    public void dropWobble() {
        hook.setPosition(0);
        sleep(750);
        arm.setPosition(1);
        sleep(750);
        arm.setPosition(0);
        hook.setPosition(1);
        sleep(250);
        hook.setPosition(1);

    }

    public void lowerWobble() {
        hook.setPosition(0);
        sleep(250);
        arm.setPosition(1);
    }

    public void raiseWobble() {
        arm.setPosition(0);
        sleep(500);
        hook.setPosition(1);
    }

    /**
     *
     */
    public void turnOnShooter(double power) {
        shootR.setPower(power);
        shootL.setPower(power);
    }

    /**
     *
     */
    public void turnOnIntake(double power) {
        intake.setPower(power);
        feeder.setPower(power);
    }

    /**
     *
     */
    public void turnOffShooter() {
        shootL.setPower(0);
        shootR.setPower(0);
    }

    public void turnOffIntake() {
        intake.setPower(0);
        feeder.setPower(0);
    }

    /**
     * shoots my shot
     *
     * @param powerIntake  intake speed
     * @param powerShooter shooter speed
     * @throws InterruptedException
     */
    public void shoot(double powerShooter, double powerIntake) throws InterruptedException {
        double startTime = runtime.time();
//        turnOnShooter(powerShooter);
        shootR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shootL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shootL.setPower(powerShooter);
        shootR.setPower(powerShooter);
//        sleep(4000);
        while (4 + startTime > runtime.time()) {
            heartbeat();
            System.out.println(shootL.getVelocity());
            System.out.println(shootR.getVelocity());

        }
        startTime += 4;
        turnOnIntake(powerIntake);

        while (8 + startTime > runtime.time()) {
            heartbeat();
            System.out.println(shootL.getVelocity());
            System.out.println(shootR.getVelocity());

        }
        turnOffIntake();
        turnOffShooter();
        shootR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shootL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


    }

    /**
     * @param xcoords
     * @param ycoords
     * @param maximumPower
     * @param initPower
     * @param finalPower
     * @param offset
     * @param halt
     * @throws InterruptedException
     */
    public void splineMove(double[] xcoords, double[] ycoords, double maximumPower, double initPower, double finalPower, double offset, boolean halt) throws InterruptedException {
        double baseParallelLeftTicks = shootL.getCurrentPosition();
        double baseParallelRightTicks = -shootR.getCurrentPosition();

        double parallelLeftTicks = 0;
        double parallelRightTicks = 0;

        double sRight = 0;
        double sLeft = 0;
        double sAvg = 0;

        //double startingPosition = parallelEncoderTracker.getCurrentPosition();
        Waypoint[] coords = new Waypoint[xcoords.length];
        boolean inverted = false;

        //set Waypoints per each (x,y)
        for (int i = 0; i < coords.length; i++) {
            coords[i] = new Waypoint(xcoords[i], ycoords[i]);
        }

        //if spline backwards, set inverted to true (lets correction method know to make adjustments to targetHeading in PD correction method)
        if (maximumPower < 0) {
            inverted = true;
        }

        //sets new spline, defines important characteristics
        Bezier spline = new Bezier(coords);
        double t = 0;
        double distanceTraveled;
        double arclength = spline.getArcLength(); //computes arc length by adding infinitesimally small slices of sqrt( (dx/dt)^2 + (dy/dt)^2 ) (distance formula). This method uses integration, a fundamental component in calculus

        motionProfiler = new MotionProfiler(.125);
        double currentPower = 0;

        double time = runtime.time();

        while (t <= 1.0 && runtime.time() - time < 7) {
            heartbeat();

            currentPower = motionProfiler.getProfilePower(t, maximumPower, initPower, finalPower);
            //constantly adjusts heading based on what the current spline angle should be based on the calculated t
//            correction(currentPower, (int) (Math.toDegrees(spline.getAngle(t, offset))), "spline", inverted, 1.0);
            //distanceTraveled computed by converting encoderTraveled ticks on deadwheel to inches traveled
            parallelLeftTicks = shootL.getCurrentPosition() - baseParallelLeftTicks;
            parallelRightTicks = -shootR.getCurrentPosition() - baseParallelRightTicks;

            sRight = (parallelRightTicks) * 1 / DEAD_WHEEL_TO_TICKS;
            sLeft = (parallelLeftTicks) * 1 / DEAD_WHEEL_TO_TICKS;
            sAvg = (sLeft + sRight) / 2;

            distanceTraveled = sAvg;

            //positionTracker.updateTicks(parallelLeftTicks, parallelRightTicks, perpendicularTicks);
            //positionTracker.updateLocationAndPose(telemetry, "spline");

            t = Math.abs(distanceTraveled / arclength);
        }
        if (halt) {
            halt();
        }
    }


    /**
     * @param arr
     * @return
     */
    public double getMaxMagnitude(double[] arr) {
        double max = Math.abs(arr[0]);

        for (int i = 1; i < arr.length; i++) {
            if (Math.abs(arr[i]) > max) {
                max = arr[i];
            }
        }

        return max;
    }

    /**
     * @param power
     * @param targetHeading
     * @param movementType
     * @param inverted
     * @param max
     */
    public void correction(double power, double targetHeading, String movementType, boolean inverted, double max) {
        //sets target and current angles
        double target = targetHeading;
        double current = getCurrentAngle();

        //if the spline motion is backwards, the target must be flipped 180 degrees in order to match with spline.getAngle()
        if (inverted && movementType.contains("spline")) {
            target = (targetHeading > 0) ? (targetHeading - 180) : (targetHeading + 180);
        }

        //when axis between -179 and 179 degrees is crossed, degrees must be converted from 0 - 360 degrees. 179-(-179) = 358. 179 - 181 = -2. Big difference
        double error = getError(current, target);


        //PD correction for both regular and spline motion
        if (movementType.contains("straight") || movementType.contains("spline")) {
            double correction = pid.correction(error, runtime);

            double leftPower = Range.clip(power - correction, -max, max);
            double rightPower = Range.clip(power + correction, -max, max);

            leftFront.setPower(leftPower);
            rightFront.setPower(rightPower);
            leftRear.setPower(leftPower);
            rightRear.setPower(rightPower);
//            telemetry.addData("left expected power", leftPower);
//            telemetry.addData("right expected power", rightPower);
//            telemetry.addData("actual left power", leftFront.getPower());
//            telemetry.addData("actual right power", rightFront.getPower());
        }

        //pd correction for strafe motion. Right and left are opposites
        else if (movementType.contains("strafe")) {
            double correction = pidStrafe.correction(error, runtime);

            if (movementType.contains("left")) {
                leftFront.setPower(Range.clip(-power - correction, -1.0, 1.0));
                rightFront.setPower(Range.clip(power + correction, -1.0, 1.0));
                leftRear.setPower(Range.clip(power - correction, -1.0, 1.0));
                rightRear.setPower(Range.clip(-power + correction, -1.0, 1.0));
            } else if (movementType.contains("right")) {
                leftFront.setPower(Range.clip(power - correction, -1.0, 1.0));
                rightFront.setPower(Range.clip(-power + correction, -1.0, 1.0));
                leftRear.setPower(Range.clip(-power - correction, -1.0, 1.0));
                rightRear.setPower(Range.clip(power + correction, -1.0, 1.0));
            }
        }

//        telemetry.addData("current Angle", current);
//        telemetry.addData("target", target);
//        telemetry.addData("error", error);
//        telemetry.addData("lf", leftFront.getPower());
//        telemetry.addData("lb", leftBack.getPower());
//        telemetry.addData("rf", rightFront.getPower());
//        telemetry.addData("rb", rightBack.getPower());
//        telemetry.addData("avg power", (rightBack.getPower() + rightFront.getPower() + leftBack.getPower() + leftFront.getPower()) / 4);
//        telemetry.update();
    }

    public void runIntake(double t) throws InterruptedException {
        t += runtime.time();
        feeder.setPower(1);
        while (t > runtime.time()) {
            heartbeat();
        }
        feeder.setPower(0);
    }

}



