import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;


@Autonomous(name = " turning")
@Disabled
public class turning extends LinearOpMode {
    private final static double TICKS_PER_REV = 1120;
    private final static double DRIVETRAIN_WHEEL_DIAMTER = 4;
    private final static double DRIVETRAIN_GEAR_RATIO = 5 / 6;
    private final static double LINEAR_TO_TICKS = TICKS_PER_REV / (Math.PI * DRIVETRAIN_WHEEL_DIAMTER * DRIVETRAIN_GEAR_RATIO);
    private final static double radius = 7;

    private static DcMotor rightFront, leftFront, rightRear, leftRear;
    private static Servo hookL, clawL, gripL, towerBL, hookR, clawR, gripR, towerBR;
    private static DigitalChannel liftTouch;

    private static DcMotor[] drivetrain;

    @Override
    public void runOpMode() {
        leftFront = hardwareMap.dcMotor.get("leftFront");
        rightFront = hardwareMap.dcMotor.get("rightFront");
        rightRear = hardwareMap.dcMotor.get("rightRear");
        leftRear = hardwareMap.dcMotor.get("leftRear");
        //servos
        hookL = hardwareMap.servo.get("hookL");
        hookR = hardwareMap.servo.get("hookR");
        clawL = hardwareMap.servo.get("clawL");
        clawR = hardwareMap.servo.get("clawR");
        gripL = hardwareMap.servo.get("gripL");
        gripR = hardwareMap.servo.get("gripR");
        towerBL = hardwareMap.servo.get("towerBL");
        towerBR = hardwareMap.servo.get("towerBR");

        drivetrain = new DcMotor[]{rightFront, leftFront, rightRear, leftRear};

        leftFront.setDirection(DcMotor.Direction.FORWARD);
        leftRear.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        rightRear.setDirection(DcMotor.Direction.FORWARD);
        waitForStart();
        turn(90, 2, 1);
    }

    public  void turn(double degrees, int power, int direction) {
        //either 1 or -1
        // 1 for right
        // -1 for left
        double distance = radius * degrees;
        final int ticks = (int) (distance * LINEAR_TO_TICKS);
        for (DcMotor motors : drivetrain) {
            motors.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motors.setTargetPosition(ticks * direction);
            motors.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            motors.setPower(power);
        }
        while (leftRear.isBusy() && leftFront.isBusy() && rightFront.isBusy() && rightRear.isBusy()) {

        }
        for (DcMotor motors : drivetrain) {
            motors.setPower(0);
        }
    }

}
