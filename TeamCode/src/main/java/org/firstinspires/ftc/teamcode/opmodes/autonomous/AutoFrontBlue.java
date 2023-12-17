package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;


@Autonomous(
        name = "Auto - Blue Front"
)
public class AutoFrontBlue extends AutoFrontBase {

    @Override
    public void runOpMode() throws InterruptedException {

        direction=Direction.LEFT;
        doAuto();
    }

}