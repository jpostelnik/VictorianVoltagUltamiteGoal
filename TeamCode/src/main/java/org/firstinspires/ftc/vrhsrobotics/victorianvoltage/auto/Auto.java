package org.firstinspires.ftc.vrhsrobotics.victorianvoltage.auto;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.vrhsrobotics.victorianvoltage.auto.math.basicMathCalls;
import org.firstinspires.ftc.vrhsrobotics.victorianvoltage.auto.math.controltheory.KalminFilter;
import org.firstinspires.ftc.vrhsrobotics.victorianvoltage.auto.math.controltheory.PIDController;
import org.firstinspires.ftc.vrhsrobotics.victorianvoltage.auto.math.spline.Spline;
import org.firstinspires.ftc.vrhsrobotics.victorianvoltage.auto.vuforia.SkystoneDeterminationPipeline;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.io.File;


/*bsfd
* test test test*/


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


    private DcMotorEx rightFront, leftFront, rightRear, leftRear, shootR, shootL, intake, capMotor;
    private Servo elevator, arm, hook;
    private TouchSensor liftTouchBottom;

    private BNO055IMU imu;
    private ElapsedTime runtime = new ElapsedTime();

    private DcMotorEx[] driveTrain;


    //pids
    private PIDController pid = new PIDController(runtime, 1.9, 0, 0.2);
    private PIDController pidTruning = new PIDController(runtime, 1, 0, 0);
    private PIDController pidStrafe = new PIDController(runtime, 1, 0, 0);
    private PIDController pidDiagonal = new PIDController(runtime, 0.01, 0, 0);

    protected WebcamName weCam;
    protected OpenCvCamera camera;
    protected SkystoneDeterminationPipeline pipeline;


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


    private void intSensors(){
        liftTouchBottom = hardwareMap.touchSensor.get("liftTouchBottom");
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

        shootL.setDirection(DcMotor.Direction.REVERSE);
        shootR.setDirection(DcMotor.Direction.REVERSE);

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

        telemetry.addLine("deadwheels done");
        telemetry.update();

    }

    /**
     * inits servos
     */
    private void initServos() {
        elevator = hardwareMap.servo.get("elevator");
        elevator.setPosition(1);

        arm = hardwareMap.servo.get("arm");
        arm.setPosition(0);

        hook = hardwareMap.servo.get("hook");
        hook.setPosition(1);
    }

    protected void intCamera(){
        weCam = hardwareMap.get(WebcamName.class, "Webcam 1");
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(weCam, cameraMonitorViewId);
        pipeline = new SkystoneDeterminationPipeline();
        camera.setPipeline(pipeline);

        // We set the viewport policy to optimized view so the preview doesn't appear 90 deg
        // out when the RC activity is in portrait. We do our actual image processing assuming
        // landscape orientation, though.
//        camera.setViewportRenderingPolicy(OpenCvCamera.ViewportRenderingPolicy.OPTIMIZE_VIEW);

        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(320,240, OpenCvCameraRotation.SIDEWAYS_LEFT);
            }
        });
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


//    private static final String VUFORIA_KEY =
//            "AXmG74D/////AAABmcbI+OuRcECkskgZxnebQ4aCvV3y4ZxNNMqL/ii0UTZ4LvJzrZXQ/Tmpjus37PyY6Qgy2esiZm1gCbpD08BG3bplvN1aDfRWlrXuhnwbsXfRT1WoJlg41K1j3jEY3+JMn3nQ0dFslzFDomXDRe9PUpuEPyZpR2uCkmWT26JOIfImG0kkdgTmYnxiuVCwE5k4qfYGZq0qxx6q5OowqkB/WLcMB9lGD5b88oGOMDXoil0JI4pZcVam4fdERnd490N9pX7mzdXYfDPntu+uYKZu9kNHbU6rqrnJJfzX3C0WXa1qF2e3zJCyR5ckciG/I4fSZgISyPwmMPO3+ss0NcboYEnPQvsG8Onwu30/Qq42B5/O";
//
//
//    /**
//     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
//     * localization engine.
//     */
//    private VuforiaLocalizer vuforia;
//
//    /**
//     * {@link #tfod} is the variable we will use to store our instance of the TensorFlow Object
//     * Detection engine.
//     */
//    private TFObjectDetector tfod;
//
//    /**
//     *
//     */
//    private void initVuforia() {
//        /*
//         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
//         */
//        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();
//
//        parameters.vuforiaLicenseKey = VUFORIA_KEY;
//        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");
//
//        //  Instantiate the Vuforia engine
//        vuforia = ClassFactory.getInstance().createVuforia(parameters);
//
//        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
//    }
//
//    private static final String TFOD_MODEL_ASSET = "UltimateGoal.tflite";
//    private static final String LABEL_FIRST_ELEMENT = "Quad";
//    private static final String LABEL_SECOND_ELEMENT = "Single";
//
//    /**
//     * Initialize the TensorFlow Object Detection engine.
//     */
//    private void initTfod() {
//        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
//                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
//        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
//        tfodParameters.minResultConfidence = 0.8f;
//        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
//        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
//    }
//
//    /**
//     * this initialize every motor, servo, and sensor
//     */
//
//    protected void initVision() {
//        initVuforia();
//        initTfod();
//        if (tfod != null) {
//            tfod.activate();
//        }
//    }

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
    private void halt() {
        for (DcMotorEx motor : driveTrain) {
            motor.setPower(0);
        }
    }


//    public String getObjectAmount() {
//        List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
//        String a = "";
//        for (int i = 0; i < 100; i++) {
//            for (Recognition recognition : updatedRecognitions) {
//                a =
//                        recognition.getLabel();
//                telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
//                telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
//                        recognition.getLeft(), recognition.getTop());
//                telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
//                        recognition.getRight(), recognition.getBottom());
//            }
//            telemetry.update();
//        }
////        telemetry.addData("# Object Detected", updatedRecognitions.size());
//        telemetry.update();
//        return a;
//    }


    @Deprecated
    /**
     * @param distance distance in inches that the robot move
     * @param power    it is how fast the robot moves
     * @param heading  the target heading for the front of the robot
     * @throws InterruptedException if op mod is not running then it stops
     */
    public void move(double distance, double power, int heading, ElapsedTime runtime) throws InterruptedException {
        resetMotors();
        pid.reset(runtime);
        double deadWheelStart = -1*shootR.getCurrentPosition();
        ;
        double currentPosition = leftRear.getCurrentPosition();
        int ticks = basicMathCalls.getTicksMotor(distance, "linear");
//        int ticks = (int) (distance * LINEAR_TO_TICKS);
        telemetry.clear();
        telemetry.addData("starting distance",deadWheelStart/DEAD_WHEEL_TO_TICKS);
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
            System.out.println("distance traveled "+(-1*shootR.getCurrentPosition()-deadWheelStart)/DEAD_WHEEL_TO_TICKS);
            telemetry.addData("distance traveled", (shootR.getCurrentPosition()-deadWheelStart)/DEAD_WHEEL_TO_TICKS);
            correction(heading, "strait", power, 1);
            heartbeat();
            telemetry.update();

        }
        telemetry.addData("ending position", leftRear.getCurrentPosition());
        telemetry.addData("distance traveled", (-1*shootR.getCurrentPosition()-deadWheelStart)/DEAD_WHEEL_TO_TICKS);
        telemetry.update();


        halt();
    }

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
            correction(heading, "strait", power, 1);
            heartbeat();
        }
        halt();
    }

    public void strafeByDeadWheels(double distance, double power, boolean right, int heading, ElapsedTime runtime) throws InterruptedException {
        resetMotors();
        resetDeadWheels();
        pidStrafe.reset(runtime);
        double currentPosition = intake.getCurrentPosition();

        int ticks = basicMathCalls.getDeadWheelTicks(distance);
        String direction = "";
        if (right) {
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
            correction(heading, "strafe" + direction, power, 1);
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
    @Deprecated
    /**
     * moves the robot in a lateral direction
     *
     * @param distance where i want to go
     * @param power    the speed
     * @param right    direction
     * @throws InterruptedException
     */
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
    @Deprecated
    /**
     * used for turning just by ticks
     *
     * @param power     sets the speed
     * @param degrees   degree of a circle
     * @param direction either "right" or "left"
     * @throws InterruptedException throws if heartbeat throws
     */
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
            error = KalminFilter.getError(current, target);

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
        double error = KalminFilter.getError(current, target);
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
        double error = KalminFilter.getError(current, targetHeading);

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
     * shoots my shot
     *
     * @param power power of the motors. can be tuned if shooting to high or low
     * @param time  how long i want the motors running for.
     * @throws InterruptedException
     */
    public void shoot(double power, double time) throws InterruptedException {
        time += runtime.time();
        while (time > runtime.time()) {
            shootR.setPower(power);
            shootL.setPower(power);
            heartbeat();
        }
        shootL.setPower(0);
        shootR.setPower(0);

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

    public void printTicks(){
        telemetry.clear();
        telemetry.addData("forward right", shootR.getCurrentPosition() / DEAD_WHEEL_TO_TICKS);
        telemetry.addData("forward Left",shootL.getCurrentPosition()/DEAD_WHEEL_TO_TICKS);
        telemetry.addData("horizontal", intake.getCurrentPosition() / DEAD_WHEEL_TO_TICKS);
        telemetry.addData("Right Front ticks",rightFront.getCurrentPosition()/LINEAR_TO_TICKS);
        telemetry.addData("Left Front ticks",leftFront.getCurrentPosition()/LINEAR_TO_TICKS);
        telemetry.addData("Right Rear ticks",rightRear.getCurrentPosition()/LINEAR_TO_TICKS);
        telemetry.addData("Left Rear ticks",leftRear.getCurrentPosition()/LINEAR_TO_TICKS);
        telemetry.addData("velocity",rightFront.getVelocity());


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
}



