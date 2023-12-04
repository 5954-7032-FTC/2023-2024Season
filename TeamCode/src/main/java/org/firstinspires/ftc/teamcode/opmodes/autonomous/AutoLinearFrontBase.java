package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.Constants;
public class AutoLinearFrontBase extends AutoLinearBase {


    public void doAuto() throws InterruptedException {

        initRobot();

        telemetry.update();
        _pixelDeliveryThread.start();
        waitForStart();

        _pixelDelivery.extendSensors();

        // drive almost to the pieces
        moveDirection(30);
        // start looking for a piece front or back

        _pixelDeliveryThread.setLooking(true);
        moveDirection(8);
        _pixelDeliveryThread.setLooking(false);
        _pixelDeliveryThread.cancel();
        // see if it saw something?
        Constants.piecePositions position = _pixelDeliveryThread.getPosition();
        Telemetry.Item T_where = telemetry.addData("Where?", "none");

        //now move back to placement location
        moveAntiDirection(14);
        switch (position) {
            case FRONT:
                T_where.setValue("front(left side)");
                if (direction==Direction.RIGHT) {
                    _pixelDelivery.rightPixelDrop(Constants.pixelDropPositions.RIGHT_FRONT);
                }
                else {
                    _pixelDelivery.leftPixelDrop(Constants.pixelDropPositions.LEFT_FRONT);
                }
                //_pixelDelivery.rightPixelDrop((direction==Direction.RIGHT)?Constants.pixelDropPositions.RIGHT_FRONT: Constants.pixelDropPositions.LEFT_FRONT);
                moveDirection(8);
                break;
            case REAR:
                T_where.setValue("rear(right side)");
                if (direction==Direction.RIGHT) {
                    _pixelDelivery.rightPixelDrop(Constants.pixelDropPositions.RIGHT_REAR);
                }
                else {
                    _pixelDelivery.leftPixelDrop(Constants.pixelDropPositions.LEFT_REAR);
                }
                //_pixelDelivery.rightPixelDrop((direction==Direction.RIGHT)?Constants.pixelDropPositions.RIGHT_REAR: Constants.pixelDropPositions.LEFT_REAR);
                moveAntiDirection(4);
                break;
            case CENTER:
                moveAntiDirection(3);
                T_where.setValue("center(center)");
                if (direction==Direction.RIGHT) {
                    _pixelDelivery.rightPixelDrop(Constants.pixelDropPositions.RIGHT_CENTER);
                }
                else {
                    _pixelDelivery.leftPixelDrop(Constants.pixelDropPositions.LEFT_CENTER);
                }
                //_pixelDelivery.rightPixelDrop((direction==Direction.RIGHT)?Constants.pixelDropPositions.RIGHT_CENTER: Constants.pixelDropPositions.LEFT_CENTER);
                moveDirection(3);
                break;
        }
        telemetry.update();

        // put arm back and wait for it.....
        try { Thread.sleep(250);} catch (Exception ignored) {}
        if (direction==Direction.RIGHT) {
            _pixelDelivery.rightPixelReset();
        }
        else {
            _pixelDelivery.leftPixelReset();
        }
        try { Thread.sleep(500);} catch (Exception ignored) {}
        _pixelDelivery.retractSensors();
        telemetry.update();

        //now place next pixel
        driveReverse(50);

        placePixel();

        driveForward(4);
        switch (position){
            case FRONT:
                moveAntiDirection(10);
                break;
            case REAR:
                break;
            case CENTER:
                moveAntiDirection(4);
                break;
        }
        moveAntiDirection(24);
        driveReverse(16);
    }

}