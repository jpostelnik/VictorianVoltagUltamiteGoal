package org.firstinspires.ftc.vrhsrobotics.victorianvoltage.auto;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
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
import org.firstinspires.ftc.vrhsrobotics.victorianvoltage.auto.exceptions.DoomException;
import org.firstinspires.ftc.vrhsrobotics.victorianvoltage.auto.exceptions.HeartBeatException;
import org.firstinspires.ftc.vrhsrobotics.victorianvoltage.auto.math.MotionProfiling.MotionProfiler;
import org.firstinspires.ftc.vrhsrobotics.victorianvoltage.auto.math.basicMathCalls;
import org.firstinspires.ftc.vrhsrobotics.victorianvoltage.auto.math.controltheory.KalmanFilter;
import org.firstinspires.ftc.vrhsrobotics.victorianvoltage.auto.math.controltheory.Kinematic;
import org.firstinspires.ftc.vrhsrobotics.victorianvoltage.auto.math.controltheory.MotionProfiling;
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

import static org.firstinspires.ftc.vrhsrobotics.victorianvoltage.auto.math.basicMathCalls.ticksToInch;
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
    private final double EPSILON = 0.25;
    private double watchDogExpiration;
    private final double WIDTH = 7;
    private final double HEIGHT = 7;

    private DcMotorEx rightFront, leftFront, rightRear, leftRear, shootR, shootL, intake, feeder;
    private DcMotorEx encX, encY;
    private Servo arm, hook, liftPlateLeft, liftPlateRight;

    private BNO055IMU imu;
    private ElapsedTime runtime = new ElapsedTime();

    private DcMotorEx[] driveTrain;
    private Rev2mDistanceSensor rightFrontDistance, leftFrontDistance;
    private MotionProfiler motionProfiler;


    //pids
    private PIDController pid = new PIDController(runtime, 0.0004, 0, 0.0024);
    private PIDController pidTurning = new PIDController(runtime, 1, 0, 0);
    private PIDController pidStrafe = new PIDController(runtime, 0.04, 0, 0.004);
    private PIDController pidDiagonal = new PIDController(runtime, 0.01, 0, 0);
    private PIDController pidPositional = new PIDController(runtime, 0.7, 0.1, 0);

    protected WebcamName weCam;
    protected OpenCvCamera camera;
    protected SkystoneDeterminationPipeline pipeline;

    private Kinematic robotKinematics = new Kinematic(WIDTH, HEIGHT);

    //KalmanFilter Filter
    //updates ever 50 miliseconds
    double dt = 0.025;
    SimpleMatrix F = new SimpleMatrix(new double[][]{
            {1, 0, dt, 0},
            {0, 1, 0, dt},
            {0, 0, 1, 0},
            {0, 0, 0, 1}
    });

    SimpleMatrix G = new SimpleMatrix(new double[][]{
            {0.5 * dt * dt},
            {0.5 * dt * dt},
            {dt},
            {dt}
    });

    double sigma_a = 0.005;
    SimpleMatrix Q = G.mult(G.transpose()).scale(sigma_a * sigma_a);

    SimpleMatrix H = SimpleMatrix.identity(F.numRows());

    double positionVar = 0.005;
    double velocityVar = 0.001;

    SimpleMatrix R = new SimpleMatrix(new double[][]{
            {positionVar, 0, 0, 0},
            {0, positionVar, 0, 0},
            {0, 0, velocityVar, 0},
            {0, 0, 0, velocityVar}
    });


    double a_x = 20;
    double a_y = 20;

    SimpleMatrix u_u = new SimpleMatrix(new double[][]{
            {a_x},
            {a_y},
            {a_x},
            {a_y}
    });

    SimpleMatrix u_0 = new SimpleMatrix(new double[][]{
            {0},
            {0},
            {0},
            {0}
    });

    KalmanFilter robotKalmanFilter = new KalmanFilter(F, G, R, Q, H);


    /**
     * checks if opmode is still running
     *
     * @throws HeartBeatException
     */
    private void heartbeat() throws HeartBeatException {
        if (!opModeIsActive()) {
            System.out.println("op mode failed");
            telemetry.addLine("op mode failed");
            telemetry.update();
            throw new HeartBeatException();
        }
        if (runtime.seconds() > watchDogExpiration) {
            System.out.println("watch dog expired");
            telemetry.addLine("watch dog expired");
            telemetry.update();
            throw new HeartBeatException();
        }
    }


    private void doom(boolean test) throws DoomException {
        if (false) {
            throw new DoomException();
        }
    }

    public void setWatchDogExpiration(double watchDogExpiration) {
        this.watchDogExpiration = watchDogExpiration + runtime.seconds();
    }

    /**
     *
     */
    protected void resetRuntime() {
        runtime.reset();
    }

    /**
     * currentPositionVector
     */
    private void intSensors() {
        leftFrontDistance = (Rev2mDistanceSensor) hardwareMap.get("leftFrontDistance");
        rightFrontDistance = (Rev2mDistanceSensor) hardwareMap.get("rightFrontDistance");
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

        shootR.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        intake.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        shootL.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        feeder.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        //alias to mark as deadwheels
        encX = intake;
        encY = shootR;

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
     * this rests the encoder positions to 0
     */
    private void resetMotors() {
        for (DcMotorEx motor : driveTrain) {
            motor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        }
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
        drive(0);
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
     * @param distance distance in inches that the robot move
     * @param power    it is how fast the robot moves
     * @param heading  the target heading for the front of the robot
     * @throws HeartBeatException if op mod is not running then it stops
     *                            oof
     */
    @Deprecated
    public void moveOld(double distance, double power, int heading, ElapsedTime runtime) throws HeartBeatException {
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
        drive(power);
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
     * @throws HeartBeatException
     */
    public void moveByDeadWheels(double distance, double power, int heading, ElapsedTime runtime) throws HeartBeatException {
        pid.reset(runtime);
        resetDeadWheels();
        double currentPosition = shootR.getCurrentPosition();
        int ticks = basicMathCalls.getDeadWheelTicks(distance);
        drive(power);
        while (Math.abs(shootR.getCurrentPosition() - currentPosition) < ticks) ;
        {
            correction(power, heading, "straight", false, 1);
            heartbeat();
        }
        halt();
    }

    /**
     * @param time
     * @param power
     * @param runtime
     * @throws HeartBeatException
     */
    @Deprecated
    protected void moveByTime(double time, double power, ElapsedTime runtime) throws HeartBeatException {
        resetMotors();
        double statingPosition = leftRear.getCurrentPosition();

        double endTime = runtime.seconds() + time;

        drive(power);
        while (runtime.seconds() < endTime) {
            printTicks();
            heartbeat();
        }
        halt();
    }


    public void move(SimpleMatrix target, double power, ElapsedTime runtime) throws HeartBeatException {
        pid.reset(runtime);
        resetDeadWheels();
        setWatchDogExpiration(target.normF() / 25);
        MotionProfiling mp = new MotionProfiling(power,robotKinematics);

        double theta = 0;
        SimpleMatrix B = new SimpleMatrix(new double[][]{
                {dt * dt / 2 * Math.cos(theta), 0, 0, 0},
                {0, dt * dt / 2 * Math.sin(theta), 0, 0},
                {0, 0, dt * dt / 2 * Math.cos(theta), 0},
                {0, 0, 0, dt * dt / 2 * Math.sin(theta)}

        });

        SimpleMatrix x_0 = new SimpleMatrix(new double[][]{
                {0},
                {0},
                {0},
                {0}
        });
        SimpleMatrix p_0 = new SimpleMatrix(new double[][]{
                {0.1, 0, 0, 0},
                {0, 0.1, 0, 0},
                {0, 0, 0.1, 0},
                {0, 0, 0, 0.1}
        });
        robotKalmanFilter.setInitalPostion(x_0, p_0, B);


        double powerSteps;
        if (power > 0) {
            powerSteps = 0.05;
        } else {
            powerSteps = -0.05;
        }
        power = Math.abs(power);


        SimpleMatrix updatedEst = x_0;

        double nextUpdateTime = runtime.milliseconds() + dt * 1000;
        double lastEstimateTime = runtime.seconds();
        double lastX = ticksToInch(encX.getCurrentPosition()),
                lastY = ticksToInch(encY.getCurrentPosition());

        while (true) {
            SimpleMatrix error = target.minus(position(updatedEst));
            double d = error.normF();
            System.out.println("distance = " + d);
            if (d < EPSILON) {
                break;
            }
            heartbeat();

            drive(mp.power(error));
//            drive(mp.power());

            double sleepTime = nextUpdateTime - runtime.milliseconds();
            if (sleepTime > 0) {
                sleep((long) sleepTime);
            }
            nextUpdateTime += dt * 1000;

            double x = ticksToInch(encX.getCurrentPosition()),
                    y = ticksToInch(encY.getCurrentPosition());
            double dt_actual = runtime.seconds() - lastEstimateTime;
            lastEstimateTime = runtime.seconds();



            SimpleMatrix z_k = new SimpleMatrix(new double[][]{
                    {x},
                    {y},
                    {(x - lastX) / dt_actual},
                    {(y - lastY) / dt_actual}
            });
            lastX = x;
            lastY = y;

            SimpleMatrix v = velocity(z_k);
            double speed = v.normF();
            SimpleMatrix u = estimateControlInput(speed, 50);
            updatedEst = robotKalmanFilter.update(z_k, u);

            System.out.println("dt_actual = " + dt_actual);
            System.out.println("speed = " + speed);
            System.out.println("z_k = " + z_k.transpose());
            System.out.println("updatedEst = " + updatedEst.transpose());
            telemetry.update();

            //saftey checks
//          doom(leftFrontDistance.getDistance(DistanceUnit.CM) < 10 || rightFrontDistance.getDistance(DistanceUnit.CM) < 10);
        }

        halt();
    }

    private SimpleMatrix estimateControlInput(double speed, int steadyStateSpeed) {
        SimpleMatrix u;
        if (speed < steadyStateSpeed) {
            u = u_u;
        } else {
            u = u_0;
        }
        return u;
    }

    private void drive(double currentPower) {
        for (DcMotorEx motor : driveTrain) {
            motor.setPower(currentPower);
        }
    }

    private void drive(SimpleMatrix w) {
        rightFront.setPower(w.get(0));
        leftRear.setPower(w.get(2));
        leftFront.setPower(w.get(1));
        rightRear.setPower(w.get(3));

    }

    private SimpleMatrix position(SimpleMatrix updatedEst) {
        return updatedEst.extractMatrix(0, 2, 0, 1);
    }

    private SimpleMatrix velocity(SimpleMatrix updatedEst) {
        return updatedEst.extractMatrix(2, 4, 0, 1);
    }

    public void strafe(double distance, double power, boolean left, int heading, ElapsedTime runtime) throws HeartBeatException {
        pid.reset(this.runtime);
        resetDeadWheels();

        double theta = Math.PI / 2;
        SimpleMatrix B = new SimpleMatrix(new double[][]{
                {dt * dt / 2 * Math.cos(theta), 0, 0, 0},
                {0, dt * dt / 2 * Math.sin(theta), 0, 0},
                {0, 0, dt * dt / 2 * Math.cos(theta), 0},
                {0, 0, 0, dt * dt / 2 * Math.sin(theta)}

        });
        robotKalmanFilter.setInitalPostion(new SimpleMatrix(new double[][]{
                        {0},
                        {0},
                        {0},
                        {0}
                }),
                new SimpleMatrix(new double[][]{
                        {0.1, 0, 0, 0},
                        {0, 0.1, 0, 0},
                        {0, 0, 0.1, 0},
                        {0, 0, 0, 0.1}
                }),
                B);

        double currentPosition = shootR.getCurrentPosition();
        int ticksX = basicMathCalls.getDeadWheelTicks(0);
        int ticksY = basicMathCalls.getDeadWheelTicks(distance);
        leftRear.setPower(-1 * power);
        leftFront.setPower(1 * power);
        rightRear.setPower(1 * power);
        rightFront.setPower(-1 * power);
        SimpleMatrix targetVector = new SimpleMatrix(new double[][]{
                {0},
                {distance}
        });
        double errorMagnitude = 1000000;
        int z = (int) (2 * DEAD_WHEEL_TO_TICKS);

        int i = 1;
        double powerSteps;
        if (power < -1) {
            powerSteps = -0.025;

        } else {
            powerSteps = 0.025;

        }
        power = Math.abs(power);
        if (left) {
            leftRear.setPower(1 * powerSteps);
            leftFront.setPower(-1 * powerSteps);
            rightRear.setPower(-1 * powerSteps);
            rightFront.setPower(1 * powerSteps);
//            direction = "right";
        } else {
            leftRear.setPower(-1 * powerSteps);
            leftFront.setPower(1 * powerSteps);
            rightRear.setPower(1 * powerSteps);
            rightFront.setPower(-1 * powerSteps);
//            direction = "left";
        }
        while (Math.abs(intake.getCurrentPosition() - currentPosition) < ticksY) {
            double start = this.runtime.milliseconds();

            SimpleMatrix z_k = new SimpleMatrix(new double[][]{
                    {ticksToInch(encX.getCurrentPosition())},
                    {ticksToInch(encY.getCurrentPosition())},
                    {ticksToInch(encX.getVelocity())},
                    {ticksToInch(encY.getVelocity())}
            });
            SimpleMatrix updatedEst = estimateControlInput(basicMathCalls.ticksToInch(encY.getVelocity()), 35);
            SimpleMatrix currentPositionVector = new SimpleMatrix(new double[][]{
                    {0},
                    {0}
            });

            SimpleMatrix errorVector = targetVector.minus(currentPositionVector);
            errorMagnitude = Math.sqrt(Math.pow(errorVector.get(0, 0), 2) + Math.pow(errorVector.get(0, 0), 2));
            double correction = pidPositional.correction(errorMagnitude, this.runtime);
            if (left) {
                correction(Range.clip(powerSteps * i, -power, power), heading, "strafe left", false, 1);
            } else {
                correction(Range.clip(powerSteps * i, -power, power), heading, "strafe right", false, 1);

            }
//            leftRear.setPower(-1 * power);
//            leftFront.setPower(1 * power);
//            rightRear.setPower(1 * power);
//            rightFront.setPower(-1 * power);
            double end = this.runtime.milliseconds();
            double elapsedTime = end - start;
            double sleepTime = (50 - elapsedTime);
//            sleep((long) sleepTime);
            System.out.println("elapsedTime = " + elapsedTime);
            System.out.println("z_k = " + z_k.transpose());
            System.out.println("updatedEst = " + updatedEst.transpose());
//            telemetry.addData("current", currentPositionVector);
//            System.out.println("CurrentVector = "+currentPositionVector.transpose());
//            telemetry.addData("errorVector", errorVector);
//            System.out.println("Error vector = "+errorVector.transpose());

            telemetry.update();
            heartbeat();
        }
        halt();
    }

    private double distance(SimpleMatrix x1, SimpleMatrix x2) {
        return x1.minus(x2).normF();
    }

    /**
     * @param distance
     * @param power
     * @param left
     * @param heading
     * @param runtime
     * @throws HeartBeatException
     */
    public void strafeByDeadWheels(double distance, double power, boolean left, int heading, ElapsedTime runtime) throws HeartBeatException {
        resetMotors();
        resetDeadWheels();
        pidStrafe.reset(runtime);
        double currentPosition = intake.getCurrentPosition();
        int ticks = basicMathCalls.getDeadWheelTicks(distance);
        String direction = "";
//        int i = 1;
//        double powerSteps;
//        if (power > 0) {
//            powerSteps = 0.05;
//        } else {
//            powerSteps = -0.05;
//        }


        if (left) {
            leftRear.setPower(1 * power);
            leftFront.setPower(-1 * power);
            rightRear.setPower(-1 * power);
            rightFront.setPower(1 * power);
//            leftFront.setPower(Range.clip(-powerSteps * i, -1, 1));
//            leftRear.setPower(Range.clip(powerSteps * i, -1, 1));
//            rightRear.setPower(Range.clip(-powerSteps * i, -1, 1));
//            rightFront.setPower(Range.clip(powerSteps * i, -1, 1));
            direction = "right";
        } else {
            leftRear.setPower(-1 * power);
            leftFront.setPower(1 * power);
            rightRear.setPower(1 * power);
            rightFront.setPower(-1 * power);
//            leftFront.setPower(Range.clip(powerSteps * i, -1, 1));
//            leftRear.setPower(Range.clip(-powerSteps * i, -1, 1));
//            rightRear.setPower(Range.clip(powerSteps * i, -1, 1));
//            rightFront.setPower(Range.clip(-powerSteps * i, -1, 1));
            direction = "left";
        }

        while (Math.abs(intake.getCurrentPosition() - currentPosition) < ticks) ;
        {
            correction(power, heading, "strafe" + direction, false, 1);
            heartbeat();
        }
        halt();
    }


    /**
     * moves the robot in a lateral direction
     *
     * @param distance where i want to go
     * @param power    the speed
     * @param right    direction
     * @throws HeartBeatException
     * @deprecated - use strafebyDeadwheels
     */
    @Deprecated
    protected void strafeOld(double distance, double power, boolean right, int heading, ElapsedTime runtime) throws HeartBeatException {
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

    public void diagonalStrafe(double xDistance, double yDistance, double power, int heading, ElapsedTime runtime) throws HeartBeatException {
        pid.reset(this.runtime);
        resetDeadWheels();

        double theta = basicMathCalls.getAngleDegrees(xDistance, yDistance);
        SimpleMatrix B = new SimpleMatrix(new double[][]{
                {dt * dt / 2 * Math.cos(theta), 0, 0, 0},
                {0, dt * dt / 2 * Math.sin(theta), 0, 0},
                {0, 0, dt * dt / 2 * Math.cos(theta), 0},
                {0, 0, 0, dt * dt / 2 * Math.sin(theta)}

        });
        robotKalmanFilter.setInitalPostion(new SimpleMatrix(new double[][]{
                        {0},
                        {0},
                        {0},
                        {0}
                }),
                new SimpleMatrix(new double[][]{
                        {0.1, 0, 0, 0},
                        {0, 0.1, 0, 0},
                        {0, 0, 0.1, 0},
                        {0, 0, 0, 0.1}
                }),
                B);
        resetDeadWheels();
        double currentPosition = shootR.getCurrentPosition();
        int ticksX = basicMathCalls.getDeadWheelTicks(xDistance);
        int ticksY = basicMathCalls.getDeadWheelTicks(yDistance);

        double degrees = basicMathCalls.getAngleDegrees(xDistance, yDistance);
        double secondaryPower = basicMathCalls.getSecondPower(degrees, power);

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


        SimpleMatrix targetVector = new SimpleMatrix(new double[][]{
                {xDistance},
                {yDistance}
        });
        while (Math.abs(intake.getCurrentPosition() - currentPosition) < ticksY) {
            double start = this.runtime.milliseconds();

            SimpleMatrix z_k = new SimpleMatrix(new double[][]{
                    {ticksToInch(encX.getCurrentPosition())},
                    {ticksToInch(encY.getCurrentPosition())},
                    {ticksToInch(encX.getVelocity())},
                    {ticksToInch(encY.getVelocity())}
            });
            SimpleMatrix updatedEst = estimateControlInput(basicMathCalls.ticksToInch(encY.getVelocity()), 35);

            SimpleMatrix currentPositionVector = new SimpleMatrix(new double[][]{
                    {0},
                    {0}
            });

            SimpleMatrix errorVector = targetVector.minus(currentPositionVector);
            degrees = basicMathCalls.getAngleDegrees(xDistance, yDistance);
            secondaryPower = basicMathCalls.getSecondPower(degrees, power);

            if (degrees > 90 || degrees < -90) {
                power *= -1;
            }
            inv = false;
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


            double end = this.runtime.milliseconds();
            double elapsedTime = end - start;
            System.out.println("elapsedTime = " + elapsedTime);
            System.out.println("z_k = " + z_k.transpose());
            System.out.println("updatedEst = " + updatedEst.transpose());
//            telemetry.addData("current", currentPositionVector);
//            System.out.println("CurrentVector = "+currentPositionVector.transpose());
//            telemetry.addData("errorVector", errorVector);
//            System.out.println("Error vector = "+errorVector.transpose());

            telemetry.update();
            heartbeat();
        }
        halt();
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
     * @throws HeartBeatException
     */
    protected void diagonalStrafeOld(double xDistance, double yDistance, double power, int heading, ElapsedTime runtime) throws HeartBeatException {
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
     * used for turning just by ticks
     *
     * @param power   sets the speed
     * @param degrees degree of a circle
     * @throws HeartBeatException throws if heartbeat throws
     */
//    @Deprecated
    protected void turnTicks(double power, double degrees, ElapsedTime runtime) throws HeartBeatException {
        resetMotors();
        pidStrafe.reset(runtime);
        double currentPosition = leftRear.getCurrentPosition();
        double radians = degrees * Math.PI / 90;
        int ticks = (int) (radius * radians * LINEAR_TO_TICKS);
        rightFront.setPower(power);
        rightRear.setPower(power);
        leftRear.setPower(-1 * power);
        leftFront.setPower(-1 * power);


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
     * @throws HeartBeatException throws if heartbeat throws
     */
    protected void turningPID(double target, double powerMax, ElapsedTime runtime) throws HeartBeatException {
        PIDController pid = new PIDController(runtime, 0.9728499206, 0, 0);
        resetMotors();
        telemetry.addData("angle", getCurrentAngle());
        calibrateBnO055();
        pid.reset(runtime);
        double error = 10000;
        double current = target;
        do {
            try {
                int gyroSatus = imu.getSystemError().ordinal();
                doom(gyroSatus != 0 && gyroSatus != 1);
                current = getCurrentAngle();
                System.out.println(current);
                error = getError(current, target);

                double correction = pid.correction(error, runtime);

                leftRear.setPower(Range.clip(correction, -powerMax, powerMax));
                leftFront.setPower(Range.clip(correction, -powerMax, powerMax));
                rightRear.setPower(Range.clip(-correction, -powerMax, powerMax));
                rightFront.setPower(Range.clip(-correction, -powerMax, powerMax));

                heartbeat();
            } catch (DoomException e) {
                double renaming = current - target;
                turnTicks(powerMax, renaming, runtime);
                break;
            }
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
        System.out.println(imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYX, AngleUnit.DEGREES).firstAngle);
        System.out.println(imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).firstAngle);
        System.out.println(imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XZX, AngleUnit.DEGREES).firstAngle);
        System.out.println(imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XZX, AngleUnit.DEGREES).firstAngle);
        System.out.println(imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.YXY, AngleUnit.DEGREES).firstAngle);
        System.out.println(imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle);
        System.out.println(imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.YXZ, AngleUnit.DEGREES).firstAngle);
        System.out.println(imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.YZX, AngleUnit.DEGREES).firstAngle);
        System.out.println(imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.YZY, AngleUnit.DEGREES).firstAngle);
        System.out.println(imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle);
        System.out.println(imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYZ, AngleUnit.DEGREES).firstAngle);
        System.out.println(imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZXZ, AngleUnit.DEGREES).firstAngle);
        System.out.println(imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZXY, AngleUnit.DEGREES).firstAngle);
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

            double leftPower = Range.clip(power + correction, -max, max);
            double rightPower = Range.clip(power - correction, -max, max);

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

            if (movementType.contains("right")) {
                leftFront.setPower(Range.clip(-power - correction, -1.0, 1.0));
                rightFront.setPower(Range.clip(power + correction, -1.0, 1.0));
                leftRear.setPower(Range.clip(power - correction, -1.0, 1.0));
                rightRear.setPower(Range.clip(-power + correction, -1.0, 1.0));
            } else if (movementType.contains("left")) {
                leftFront.setPower(Range.clip(power - correction, -1.0, 1.0));
                rightFront.setPower(Range.clip(-power + correction, -1.0, 1.0));
                leftRear.setPower(Range.clip(-power - correction, -1.0, 1.0));
                rightRear.setPower(Range.clip(power + correction, -1.0, 1.0));
            }
        }

        telemetry.addData("current Angle", current);
        telemetry.addData("target", target);
        telemetry.addData("error", error);
        telemetry.addData("lf", leftFront.getPower());
        telemetry.addData("lb", leftRear.getPower());
        telemetry.addData("rf", rightFront.getPower());
        telemetry.addData("rb", rightRear.getPower());
        telemetry.addData("avg power", (rightRear.getPower() + rightFront.getPower() + leftFront.getPower() + leftFront.getPower()) / 4);
        telemetry.update();
    }


    /**
     * for printing dead wheels while testing stuff
     *
     * @throws HeartBeatException
     */
    public void printDeadWheel() throws HeartBeatException {
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
     * @param xCords
     * @param yCords
     * @param power
     * @param heading
     * @throws HeartBeatException
     */
    protected void spline(double[] xCords, double[] yCords, double power,
                          int heading, ElapsedTime runtime) throws HeartBeatException {
        diagonalStrafeOld(xCords[0], yCords[0], power, heading, runtime);
        for (int i = 1; i < xCords.length; i++) {
            diagonalStrafeOld(xCords[i] - xCords[i - 1], yCords[i] - yCords[i - 1], power, heading, runtime);
        }
    }

    /**
     * @param spline
     * @param arcLen
     * @param targetHeading
     * @param power
     * @throws HeartBeatException
     */
    public void spline(Spline spline, double arcLen, double targetHeading, double power) throws
            HeartBeatException {
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
     * @param xcoords
     * @param ycoords
     * @param maximumPower
     * @param initPower
     * @param finalPower
     * @param offset
     * @param halt
     * @throws HeartBeatException
     */
    public void splineMove(double[] xcoords, double[] ycoords, double maximumPower, double initPower, double finalPower, double offset, boolean halt) throws HeartBeatException {
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
     * @param powerShooter shooter speed
     * @param powerIntake  intake speed
     * @param waitTime
     * @throws HeartBeatException
     */
    public void shoot(double powerShooter, double powerIntake, double waitTime) throws HeartBeatException {
        double startTime = runtime.time();
//        turnOnShooter(powerShooter);
        turnOffEncoders();
        shootL.setPower(powerShooter);
        shootR.setPower(powerShooter);
//        sleep(4000);
        while (1 + startTime > runtime.time()) {
            heartbeat();
        }
        turnOnIntake(powerIntake);

        while ((waitTime + 1) + startTime > runtime.time()) {
            heartbeat();
            System.out.println(shootL.getVelocity());
            System.out.println(shootR.getVelocity());

        }
        turnOffIntake();
        turnOffShooter();
        turnOnEncoders();
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


    public void runIntake(double t) throws HeartBeatException {
        t += runtime.time();
        feeder.setPower(1);
        while (t > runtime.time()) {
            heartbeat();
        }
        feeder.setPower(0);
    }

    public void turnOffEncoders() {
        shootR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shootL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void turnOnEncoders() {
        shootR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shootL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }


}