package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;


@Autonomous(
        name = "Auto - Blue Front"
)
public class AutoLinearBlueFront extends AutoLinearBase {


    @Override
    public void runOpMode() throws InterruptedException {

        initRobot();



        waitForStart();


        // Get to position and drop first pixel
        // TODO: fix servo positions

        extendSensors();

        driveRight(30);
        if (testFrontSensor(25)) {
            //placefront
            driveForward(15);
            dropPixel();
        }
        else if (testRearSensor(25)) {
            //placerear
            driveReverse(6);
            dropPixel();
            driveForward(21);
        }
        else {
            //place center
            driveRight(6);
            dropPixel();
            driveLeft(6);
            driveForward(15);
        }

        retractSensors();

        //now place next pixel
        driveForward(39);

        dropPixel();

        driveLeft(24);



    }

}