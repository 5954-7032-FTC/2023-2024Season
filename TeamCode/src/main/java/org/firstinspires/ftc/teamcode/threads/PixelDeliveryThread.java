package org.firstinspires.ftc.teamcode.threads;

import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.PixelDelivery;
import org.firstinspires.ftc.teamcode.util.Constants;

public class PixelDeliveryThread extends RobotThread {

    PixelDelivery _pixelDelivery;

    public PixelDeliveryThread(Telemetry telemetry,
                               Servo leftPixelArm,
                               Servo leftPixelFlip,
                               Servo rightPixelArm,
                               Servo rightPixelFlip,
                               Servo[] sensorServos,
                               DistanceSensor frontSensor,
                               DistanceSensor rearSensor) {
        _pixelDelivery = new PixelDelivery(telemetry,
                leftPixelArm,
                leftPixelFlip,
                rightPixelArm,
                rightPixelFlip,
                sensorServos,
                frontSensor,
                rearSensor);
    }

    public PixelDeliveryThread(PixelDelivery pixelDelivery) {
        _pixelDelivery = pixelDelivery;
    }

    Constants.piecePositions position = Constants.piecePositions.CENTER;

    public Constants.piecePositions getPosition() {
        return position;
    }

    private boolean looking=false;

    public void setLooking(boolean looking) {
        this.looking = looking;
    }

    @Override
    public void run() {
        super.run();
        while (!isCancelled()) {
            if (!looking) continue;
            boolean front = _pixelDelivery.testFrontSensor(Constants.FRONT_SENSOR_DISTANCE_THRESHOLD);
            boolean rear = _pixelDelivery.testRearSensor(Constants.REAR_SENSOR_DISTANCE_THRESHOLD);
            if (front && !rear) {
                position = Constants.piecePositions.FRONT;
                cancel();
                break;
            }
            else if (rear && !front) {
                position = Constants.piecePositions.REAR;
                cancel();
                break;
            }
        }
    }
}
