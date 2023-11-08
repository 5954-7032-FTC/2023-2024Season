package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.subsystems.ArmSubSystem;
import org.firstinspires.ftc.teamcode.subsystems.ImuDevice;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveByGyro;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveParameters;
import org.firstinspires.ftc.teamcode.util.RobotDevices;

public class AutoLinearBase extends LinearOpMode {

    protected RobotDevices robotDevices;

    protected MecanumDriveByGyro _move;
    protected ArmSubSystem _armSubSystem;

    protected DistanceSensor frontSensor, rearSensor;

    protected Servo pixelHold0,pixelHold1;

    protected Servo [] sensorServos;


    public void initRobot() {
        telemetry.setAutoClear(false);

        robotDevices = RobotDevices.getDevices(hardwareMap);

        MecanumDriveParameters driveParameters = new MecanumDriveParameters();
        driveParameters.motors = robotDevices.wheels;
        driveParameters.ENCODER_WHEELS = new int[]{0, 1, 2, 3};
        driveParameters.REVERSED_WHEELS = new int[]{2, 3};
        driveParameters.telemetry = telemetry;
        _move = new MecanumDriveByGyro(driveParameters, new ImuDevice(robotDevices.imu));

        _armSubSystem = new ArmSubSystem(
                robotDevices.intakeServos,
                robotDevices.lowBelt,
                robotDevices.highBelt,
                robotDevices.armLift,
                robotDevices.upperArmLimit,
                robotDevices.lowerArmLimit,
                robotDevices.pixelFloor);



        frontSensor = robotDevices.frontSensor;
        rearSensor = robotDevices.rearSensor;

        pixelHold0 = robotDevices.pixelHold0;
        pixelHold1 = robotDevices.pixelHold1;

        sensorServos = robotDevices.sensorServos;

        _move.resetHeading();
    }


    @Override
    public void runOpMode() throws InterruptedException {
    //do nothing here
    }


    public  void dropPixel()  throws InterruptedException {
        // TODO: fix directions?
        pixelHold0.setPosition(0);
        pixelHold1.setPosition(1);
        Thread.sleep(150);
        pixelHold0.setPosition(1);
        pixelHold1.setPosition(0);

    }

    public void driveForward(double distanceInches) {
        _move.driveForward(distanceInches);
    }

    public void driveLeft(double distanceInches) {
        _move.driveLeft(distanceInches);
    }

    public void driveReverse(double distanceInches) {
        _move.driveReverse(distanceInches);
    }

    public void driveRight(double distanceInches) {
        _move.driveRight(distanceInches);
    }

    public void extendSensors() {
        for (Servo sensorServo: sensorServos) {
            sensorServo.setPosition(1.0);
        }
    }

    public void retractSensors() {
        for (Servo sensorServo: sensorServos) {
            sensorServo.setPosition(0.0);
        }
    }

    public boolean testFrontSensor(int mm) {
        return frontSensor.getDistance(DistanceUnit.MM) <=mm;
    }

    public boolean testRearSensor(int mm) {
        return frontSensor.getDistance(DistanceUnit.MM) <=mm;
    }

    public void placePixel() {
        _armSubSystem.raisePixelHold();
        _armSubSystem.runBeltInches(-1);
        _armSubSystem.runBeltInches(20);
        _armSubSystem.lowerPixelHold();
    }
}