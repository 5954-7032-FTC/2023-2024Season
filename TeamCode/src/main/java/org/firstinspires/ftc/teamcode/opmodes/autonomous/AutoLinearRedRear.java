package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.Constants;


@Autonomous(
        name = "Auto - Red Rear"
)
public class AutoLinearRedRear extends AutoLineaRearBase {



    @Override
    public void runOpMode() throws InterruptedException {

        direction=Direction.RIGHT;
        doAuto();

    }

}