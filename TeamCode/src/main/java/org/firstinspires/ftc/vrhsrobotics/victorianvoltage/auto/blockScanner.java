import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "Scanning")
@Disabled
public class blockScanner extends LinearOpMode {
    private final double TICKS_PER_REV = 1120;
    private final double DRIVETRAIN_WHEEL_DIAMTER = 4;
    private final double DRIVETRAIN_GEAR_RATIO = 5 / 6;
    private final double LINEAR_TO_TICKS = TICKS_PER_REV / (Math.PI * DRIVETRAIN_WHEEL_DIAMTER * DRIVETRAIN_GEAR_RATIO);

    private DcMotor rightFront, leftFront, rightRear, leftRear;
    private DcMotor[] drivetrain;

    @Override
    public void runOpMode() {
        leftFront = hardwareMap.dcMotor.get("leftFront");
        rightFront = hardwareMap.dcMotor.get("rightFront");
        rightRear = hardwareMap.dcMotor.get("rightRear");
        leftRear = hardwareMap.dcMotor.get("leftRear");

        drivetrain = new DcMotor[]{rightFront, leftFront, rightRear, leftRear};

        waitForStart();
        driveDistance(12, 0.5);


    }

    private void driveDistance(double distance, double power) {
        final int ticks = (int) (distance * LINEAR_TO_TICKS);
        for (DcMotor motor : drivetrain) {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motor.setTargetPosition(ticks);
            motor.setPower(power);
        }

        while (leftFront.isBusy() && rightFront.isBusy() && leftRear.isBusy() && rightRear.isBusy()) {
            {

            }
            for (DcMotor motor : drivetrain) {
                motor.setPower(0);
            }
        }


    }
}
