package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.Constants;

public class AutoBaseRear extends AutoBase {


    public void doAuto() throws InterruptedException {
        initRobot();

        telemetry.update();
        _pixelDeliveryThread.start();
        waitForStart();


        //_pixelDelivery.extendSensors();

        // drive almost to the pieces
        moveDirection(25); //25
        // start looking for a piece front or back

        _pixelDelivery.extendSensors();
        //magic
        _pixelDeliveryThread.setLooking(true);
        moveDirection(12); //12
        _pixelDeliveryThread.setLooking(false);
        // see if it saw something?
        _pixelDelivery.retractSensors();
        Constants.piecePositions position = _pixelDeliveryThread.getPosition();
        Telemetry.Item T_where = telemetry.addData("Where?", "none");

        //now move back to placement location
        moveAntiDirection(18);
        switch (position) {
            case FRONT:
                T_where.setValue("front(left side)");
                if (direction == Direction.RIGHT) {
                    _pixelDelivery.rightPixelDrop(Constants.pixelDropPositions.RIGHT_FRONT);
                }
                else {
                    _pixelDelivery.leftPixelDrop(Constants.pixelDropPositions.LEFT_FRONT);
                }
                //moveDirection(6);
                break;
            case REAR:
                T_where.setValue("rear(right side)");
                if (direction == Direction.RIGHT) {
                    _pixelDelivery.rightPixelDrop(Constants.pixelDropPositions.RIGHT_REAR);
                }
                else {
                    _pixelDelivery.leftPixelDrop(Constants.pixelDropPositions.LEFT_REAR);
                }
                //_pixelDelivery.rightPixelDrop(Constants.pixelDropPositions.RIGHT_REAR);
                //moveAntiDirection(4);
                break;
            case CENTER:
                T_where.setValue("center(center)");
                if (direction == Direction.RIGHT) {
                    _pixelDelivery.rightPixelDrop(Constants.pixelDropPositions.RIGHT_CENTER);
                }
                else {
                    _pixelDelivery.leftPixelDrop(Constants.pixelDropPositions.LEFT_CENTER);
                }
                break;
        }
        telemetry.update();

        // put arm back and wait for it.....
        try { Thread.sleep(250);} catch (Exception nope) {}
        if (direction==Direction.RIGHT) {
            _pixelDelivery.rightPixelReset();
        }
        else {
            _pixelDelivery.leftPixelReset();
        }
        try { Thread.sleep(500);} catch (Exception nope) {}
        _pixelDelivery.retractSensors();
        telemetry.update();

        moveAntiDirection(15);
        _armSubSystem.moveArmMillis(true,500);
        _armSubSystem.moveArmMillis(false, 500);
        // now wait 4 seconds
        pauseSeconds(4);

        //now place next pixel
        driveReverse(78);
        if (direction == Direction.RIGHT) {
            moveDirection(24);
        }
        else {
            moveDirection(14);
        }
        switch (position) {
            case FRONT:
                moveDirection(12);
                break;
            case REAR:
                break;
            case CENTER:
                moveDirection(6);
        }
        driveReverse(20);
        _move.holdHeading(Constants.DRIVE_SPEED,0,1);
        placePixel();
        try { Thread.sleep(250); } catch (InterruptedException ignore) {}
        unPlacePixel();
        try {
            Thread.sleep(250);
        }
        catch (InterruptedException ignore) {}
    }

}