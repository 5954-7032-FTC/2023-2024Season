package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.util.Constants;


@Autonomous(
        name = "Auto - Red Front"
)
public class AutoLinearRedFront extends AutoLinearFrontBase {



    @Override
    public void runOpMode() throws InterruptedException {

        direction=Direction.RIGHT;
        doAuto();

        /*
        initRobot();

        direction=Direction.RIGHT;

        telemetry.update();
        _pixelDeliveryThread.start();
        waitForStart();

        _pixelDelivery.extendSensors();

        // drive almost to the pieces
        moveDirection(30);
        // start looking for a piece front or back

        _pixelDeliveryThread.setLooking(true);
        moveDirection(12);
        _pixelDeliveryThread.setLooking(false);
        _pixelDeliveryThread.cancel();
        // see if it saw something?
        Constants.piecePositions position = _pixelDeliveryThread.getPosition();
        Telemetry.Item T_where = telemetry.addData("Where?", "none");

        //now move back to placement location
        moveAntiDirection(18);
        switch (position) {
            case FRONT:
                T_where.setValue("front(left side)");
                _pixelDelivery.rightPixelDrop(Constants.pixelDropPositions.RIGHT_FRONT);
                moveDirection(8);
                break;
            case REAR:
                T_where.setValue("rear(right side)");
                _pixelDelivery.rightPixelDrop(Constants.pixelDropPositions.RIGHT_REAR);
                moveAntiDirection(4);
                break;
            case CENTER:
                moveAntiDirection(3);
                T_where.setValue("center(center)");
                _pixelDelivery.rightPixelDrop(Constants.pixelDropPositions.RIGHT_CENTER);
                moveDirection(3);
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

         */
    }

}