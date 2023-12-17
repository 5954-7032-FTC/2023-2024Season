package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;


@Autonomous(
        name = "Auto - Red Rear - No Place"
)
public class AutoNoPlaceRearRed extends AutoNoPlaceRearBase {



    @Override
    public void runOpMode() throws InterruptedException {

        direction=Direction.RIGHT;
        doAuto();

    }

}