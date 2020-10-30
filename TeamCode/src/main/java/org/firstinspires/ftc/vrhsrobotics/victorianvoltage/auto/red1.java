package org.firstinspires.ftc.vrhsrobotics.victorianvoltage.auto;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "redWall")
public class red1 extends Auto {

    public void runOpMode(){
        initialize();
        waitForStart();
        try {


            //TODO: this will be were the scan method stuff will go, location 0 = a, 1 =b, 2 = c,
            //todo so if that happens will be after the shooter method
            int location = 0;
            move(12, 0.9, 0);
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

        }
    }
}
