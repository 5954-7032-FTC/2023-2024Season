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
        /*
        initRobot();

        telemetry.update();
        _pixelDeliveryThread.start();
        waitForStart();


        _pixelDelivery.extendSensors();

        // drive almost to the pieces
        driveRight(25);
        // start looking for a piece front or back

        //magic
        _pixelDeliveryThread.setLooking(true);
        driveRight(12);
        _pixelDeliveryThread.setLooking(false);
        // see if it saw something?
        Constants.piecePositions position = _pixelDeliveryThread.getPosition();
        Telemetry.Item T_where = telemetry.addData("Where?", "none");

        //now move back to placement location
        driveLeft(18);
        switch (position) {
            case FRONT:
                T_where.setValue("front(left side)");
                _pixelDelivery.rightPixelDrop(Constants.pixelDropPositions.RIGHT_FRONT);
                //driveRight(6);
                break;
            case REAR:
                T_where.setValue("rear(right side)");
                _pixelDelivery.rightPixelDrop(Constants.pixelDropPositions.RIGHT_REAR);
                //driveLeft(4);
                break;
            case CENTER:
                T_where.setValue("center(center)");
                _pixelDelivery.rightPixelDrop(Constants.pixelDropPositions.RIGHT_CENTER);
                break;
        }
        telemetry.update();

        // put arm back and wait for it.....
        try { Thread.sleep(250);} catch (Exception nope) {}
        _pixelDelivery.rightPixelReset();
        try { Thread.sleep(500);} catch (Exception nope) {}
        _pixelDelivery.retractSensors();
        telemetry.update();

        driveLeft(15);
        _armSubSystem.moveArmMillis(true,250);
        _armSubSystem.moveArmMillis(false, 250);
        //now place next pixel
        driveReverse(78);
        driveRight(18);
        switch (position) {
            case FRONT:
                driveRight(12);
                break;
            case REAR:
                break;
            case CENTER:
                driveRight(7);
        }
        driveReverse(20);
        placePixel();
        _pixelDeliveryThread.cancel();
         */
    }

}