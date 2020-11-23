package org.firstinspires.ftc.vrhsrobotics.victorianvoltage.auto;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "redWall")
public class red1 extends Auto {

    public void runOpMode(){
        initialize();
        telemetry.addLine("all initialized");
        telemetry.update();
        waitForStart();
        try {


            //TODO: this will be were the scan method stuff will go, location 0 = a, 1 =b, 2 = c,
            //todo so if that happens will be after the shooter method
            int location = 0;
            move(12, 1, 0);
//            shoot(1);

            switch (location){
                case 0:
                    // TODO: 9/29/20 add spline movement there
                    break;
                case 1:
                    break;
                case 2:
                    break;
            }



        }catch (Exception e)
        {
                telemetry.addLine(e.getStackTrace().toString());
                telemetry.update();
        }
    }
}
