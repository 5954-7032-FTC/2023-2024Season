package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.ArmSubSystem;
import org.firstinspires.ftc.teamcode.subsystems.ImuDevice;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveByGyro;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveParameters;
import org.firstinspires.ftc.teamcode.subsystems.PixelDelivery;
import org.firstinspires.ftc.teamcode.threads.PixelDeliveryThread;
import org.firstinspires.ftc.teamcode.util.Constants;
import org.firstinspires.ftc.teamcode.util.RobotDevices;

public class AutoLinearBase extends LinearOpMode {

    protected RobotDevices robotDevices;

    protected MecanumDriveByGyro _move;
    protected ArmSubSystem _armSubSystem;
    protected PixelDeliveryThread _pixelDeliveryThread;
    protected PixelDelivery _pixelDelivery;

    protected Telemetry.Item T_pixelHold;
    protected Telemetry.Item T_sensorServos;
    protected Telemetry.Item T_IMU;

    protected ImuDevice _imu;




    public void initRobot() {
        telemetry.setAutoClear(false);

        robotDevices = RobotDevices.getDevices(hardwareMap);


        _imu =  new ImuDevice(robotDevices.imunew);
        T_IMU = telemetry.addData("IMU", "(Yaw:(%f)",
                _imu.getHeading()
        );

        MecanumDriveParameters driveParameters = new MecanumDriveParameters();
        driveParameters.motors = robotDevices.wheels;
        driveParameters.ENCODER_WHEELS = new int[]{0, 1, 2, 3};
        driveParameters.REVERSED_WHEELS = new int[]{2, 3};
        driveParameters.telemetry = telemetry;
        _move = new MecanumDriveByGyro(driveParameters, new ImuDevice(robotDevices.imunew));
        _move.setFixHeadingToZero();

        _armSubSystem = new ArmSubSystem(
                robotDevices.intakeServos,
                robotDevices.lowBelt,
                robotDevices.highBelt,
                robotDevices.armLift,
                robotDevices.upperArmLimit,
                robotDevices.lowerArmLimit,
                robotDevices.pixelHold);


        _pixelDelivery = new PixelDelivery(
                telemetry,
                robotDevices.leftPixelArm,
                robotDevices.leftPixelFlip,
                robotDevices.rightPixelArm,
                robotDevices.rightPixelFlip,
                robotDevices.sensorServos,
                robotDevices.frontSensor,
                robotDevices.rearSensor
                );
        _pixelDeliveryThread = new PixelDeliveryThread(_pixelDelivery);

        _move.resetHeading();

        T_pixelHold = telemetry.addData("PixelHold","0=(%f),1=(%f)",0.0,0.0);
        T_sensorServos = telemetry.addData("SensorServos", "0=(%f),1=(%f)",0.0,0.0);
    }

    protected enum Direction {
        RIGHT,LEFT
    }

    protected Direction direction;
    public void moveDirection(double distanceInches) {
        switch (direction) {
            case RIGHT:
                driveRight(distanceInches);
                break;
            case LEFT:
                driveLeft(distanceInches);
                break;
        }
    }
    public void moveAntiDirection(double distanceInches) {
        switch (direction) {
            case LEFT:
                driveRight(distanceInches);
                break;
            case RIGHT:
                driveLeft(distanceInches);
                break;
        }
    }

    @Override
    public void runOpMode() throws InterruptedException {
    //do nothing here
    }

    public void driveForward(double distanceInches) {
        _move.driveForward(distanceInches*Constants.Y_DISTANCE_RATIO);

        T_IMU.setValue(_imu.getHeading());
        telemetry.update();
    }

    public void driveLeft(double distanceInches) {
        _move.driveLeft(distanceInches*Constants.X_DISTANCE_RATIO);

        T_IMU.setValue(_imu.getHeading());
        telemetry.update();
    }

    public void driveReverse(double distanceInches) {
        _move.driveReverse(distanceInches*Constants.Y_DISTANCE_RATIO);

        T_IMU.setValue(_imu.getHeading());
        telemetry.update();
    }

    public void driveRight(double distanceInches) {
        _move.driveRight(distanceInches*Constants.X_DISTANCE_RATIO);

        T_IMU.setValue(_imu.getHeading());
        telemetry.update();
    }

    public void placePixel() {
        _armSubSystem.raisePixelHold();
        _armSubSystem.runBeltMillis(false, 1200);
        _armSubSystem.lowerPixelHold();
    }

    public void unPlacePixel() {
        _armSubSystem.raisePixelHold();
        _armSubSystem.runBeltMillis(true, 1200);
    }

}