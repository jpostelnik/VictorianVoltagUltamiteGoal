package testing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import auto.Auto;

@Autonomous(name = "diagonal strafe test")
public class digonalStrafeTest extends Auto {
    @Override
    public void runOpMode() throws InterruptedException {
        initialize();
        ElapsedTime runtime = new ElapsedTime();
        waitForStart();
        runtime.reset();
        diagonalStrafe(5, 5, 0.5, 0, runtime);
        diagonalStrafe(10, 8, 0.5, 0, runtime);
        diagonalStrafe(5, 8, 0.5, 0, runtime);

    }
}
