package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;




@Autonomous(
        name = "Auto - Blue Rear"
)
public class AutoLinearBlueRear extends AutoLineaRearBase {



    @Override
    public void runOpMode() throws InterruptedException {

        direction=Direction.LEFT;
        doAuto();

    }

}