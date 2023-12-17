package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;


@Autonomous(
        name = "Auto - Red Rear"
)
public class AutoRearRed extends AutoBaseRear {



    @Override
    public void runOpMode() throws InterruptedException {

        direction=Direction.RIGHT;
        doAuto();

    }

}