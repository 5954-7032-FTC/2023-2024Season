package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.util.Constants;

public class PixelDelivery implements SubSystem{

    protected Telemetry.Item T_sensorServos;
    protected Servo _leftPixelArm, _leftPixelFlip, _rightPixelArm, _rightPixelFlip;
    protected Servo [] _sensorServos;
    protected DistanceSensor _frontSensor, _rearSensor;
    protected final String sensorFormat = "FRONT(%.1f) REAR(%.1f)";
    protected Servo _topDropServo;

    public PixelDelivery(Telemetry telemetry,
                         Servo leftPixelArm,
                         Servo leftPixelFlip,
                         Servo rightPixelArm,
                         Servo rightPixelFlip,
                         Servo[] sensorServos,
                         DistanceSensor frontSensor,
                         DistanceSensor rearSensor,
                         Servo topDropServo) {
        this._leftPixelArm = leftPixelArm;
        this._leftPixelFlip = leftPixelFlip;
        this._rightPixelArm = rightPixelArm;
        this._rightPixelFlip = rightPixelFlip;
        this._sensorServos = sensorServos;
        this._frontSensor = frontSensor;
        this._rearSensor = rearSensor;
        this.T_sensorServos = telemetry.addData("Sensors","");
        this._topDropServo = topDropServo;
    }

    public void updateTelemetry() {
        T_sensorServos.setValue(sensorFormat,_frontSensor.getDistance(DistanceUnit.MM), _rearSensor.getDistance(DistanceUnit.MM));
    }

    public boolean testRearSensor(int mm) {
        return _rearSensor.getDistance(DistanceUnit.MM) <=mm;
    }

    public double readFrontSensor() {
        return _frontSensor.getDistance(DistanceUnit.MM);
    }

    public double readRearSensor() {
        return _rearSensor.getDistance(DistanceUnit.MM);
    }

    public boolean testFrontSensor(int mm) {
        return _frontSensor.getDistance(DistanceUnit.MM) <=mm;
    }

    public void extendSensors() {
        _sensorServos[Constants.FRONT_SENSOR_SERVO].setPosition(Constants.autoSensorPositions.EXTENDED_POS.frontPos);
        _sensorServos[Constants.REAR_SENSOR_SERVO].setPosition(Constants.autoSensorPositions.EXTENDED_POS.rearPos);
    }

    public void retractSensors() {
        _sensorServos[Constants.FRONT_SENSOR_SERVO].setPosition(Constants.autoSensorPositions.RETRACTED_POS.frontPos);
        _sensorServos[Constants.REAR_SENSOR_SERVO].setPosition(Constants.autoSensorPositions.RETRACTED_POS.rearPos);
    }

    public void leftPixelReset() throws InterruptedException {
        _leftPixelFlip.setPosition(Constants.pixelDropPositions.LEFT_RESET.flipPos);
        Thread.sleep(Constants.pixelDropPositions.LEFT_RESET.ms);
        _leftPixelArm.setPosition(Constants.pixelDropPositions.LEFT_RESET.pos);
    }

    public void leftPixelResetForce() {
        _leftPixelFlip.setPosition(Constants.pixelDropPositions.LEFT_RESET.flipPos);
        _leftPixelArm.setPosition(Constants.pixelDropPositions.LEFT_RESET.pos);
    }

    public void rightPixelReset() throws InterruptedException {
        _rightPixelFlip.setPosition(Constants.pixelDropPositions.RIGHT_RESET.flipPos);
        Thread.sleep(Constants.pixelDropPositions.RIGHT_RESET.ms);
        _rightPixelArm.setPosition(Constants.pixelDropPositions.RIGHT_RESET.pos);
    }
    public void rightPixelResetForce() {
        _rightPixelFlip.setPosition(Constants.pixelDropPositions.RIGHT_RESET.flipPos);
        _rightPixelArm.setPosition(Constants.pixelDropPositions.RIGHT_RESET.pos);
    }

    public void leftPixelDrop( Constants.pixelDropPositions position) throws InterruptedException {
        _leftPixelArm.setPosition(position.pos);
        Thread.sleep(position.ms);
        _leftPixelFlip.setPosition(position.flipPos);
        Thread.sleep(position.ms);
    }

    public void rightPixelDrop( Constants.pixelDropPositions position) throws InterruptedException {
        _rightPixelArm.setPosition(position.pos);
        Thread.sleep(position.ms);
        _rightPixelFlip.setPosition(position.flipPos);
        Thread.sleep(position.ms);
    }

    public void rightPixelDrop() {
        _rightPixelArm.setPosition(Constants.pixelDropPositions.RIGHT_FRONT.pos);
        _rightPixelFlip.setPosition(Constants.pixelDropPositions.RIGHT_FRONT.flipPos);
    }

    public void leftPixelDrop() {
        _leftPixelArm.setPosition(Constants.pixelDropPositions.LEFT_FRONT.pos);
        _leftPixelFlip.setPosition(Constants.pixelDropPositions.LEFT_FRONT.flipPos);
    }

    public void topDropServoPlace() {
        _topDropServo.setPosition(Constants.topPixelPlace);

    }
    public  void  topDropServoUnPlace() {
        _topDropServo.setPosition(Constants.topPixelReset);
    }

}
