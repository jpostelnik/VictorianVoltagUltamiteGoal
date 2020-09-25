package testing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import auto.Auto;

@Autonomous(name = "spline testing")
public class splineTest extends Auto {

    private ElapsedTime runtime =new ElapsedTime();

    @Override
    public void runOpMode() {

        telemetry.addLine("starting to initialize");
        telemetry.update();
        initialize();
        double[] xCords = {5, 15, 20};
        double[] yCords = {5, 13, 21};
        waitForStart();
        runtime.reset();
        restRuntime();

        try{
            spline(xCords, yCords, 0.5, 0, runtime);
        } catch (InterruptedException e) {
        }


    }

}
