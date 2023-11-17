package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.Constants;


@Autonomous(
        name = "Auto - Red Front"
)
public class AutoLinearRedFront extends AutoLinearBase {



    @Override
    public void runOpMode() throws InterruptedException {
        initRobot();

        telemetry.update();
        waitForStart();

        _pixelDelivery.extendSensors();

        // drive almost to the pieces
        driveRight(25);
        // start looking for a piece front or back
        _pixelDeliveryThread.start();
        //magic
        driveRight(8);
        _pixelDeliveryThread.cancel();
        // see if it saw something?
        Constants.piecePositions position = _pixelDeliveryThread.getPosition();
        Telemetry.Item T_where = telemetry.addData("Where?", "none");

        //now move back to placement location
        driveLeft(11);
        switch (position) {
            case FRONT:
                T_where.setValue("front(left side)");
                _pixelDelivery.rightPixelDrop(Constants.pixelDropPositions.RIGHT_FRONT);
                driveRight(6);
                break;
            case REAR:
                T_where.setValue("rear(right side)");
                _pixelDelivery.rightPixelDrop(Constants.pixelDropPositions.RIGHT_REAR);
                driveLeft(4);
                break;
            case CENTER:
                T_where.setValue("center(center)");
                _pixelDelivery.rightPixelDrop(Constants.pixelDropPositions.RIGHT_CENTER);
                break;
        }
        telemetry.update();

        // put arm back and wait for it.....
        try { Thread.sleep(250);} catch (Exception ignored) {}
        _pixelDelivery.rightPixelReset();
        try { Thread.sleep(500);} catch (Exception ignored) {}
        _pixelDelivery.retractSensors();
        telemetry.update();

        //now place next pixel
        driveReverse(50);

        placePixel();

        driveForward(4);
        switch (position){
            case FRONT:
                driveLeft(10);
                break;
            case REAR:
                break;
            case CENTER:
                driveLeft(4);
                break;
        }
        driveLeft(24);
        driveReverse(16);
    }

}