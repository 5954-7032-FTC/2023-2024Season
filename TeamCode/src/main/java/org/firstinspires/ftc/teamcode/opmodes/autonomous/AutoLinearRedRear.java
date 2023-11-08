package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(
        name = "Auto - Red Rear"
)
public class AutoLinearRedRear extends AutoLinearBase {


    @Override
    public void runOpMode() throws InterruptedException {

        initRobot();

        waitForStart();


        // Get to position and drop first pixel
        extendSensors();

        driveLeft(30);
        if (testFrontSensor(25)) {
            //place front
            driveForward(15);
            dropPixel();
        }
        else if (testRearSensor(25)) {
            //place rear
            driveReverse(6);
            dropPixel();
            driveForward(21);
        }
        else {
            //place center
            driveLeft(6);
            dropPixel();
            driveRight(6);
            driveForward(15);
        }
        retractSensors();



        //now place next pixel
        driveForward(84);

       dropPixel();



    }

}