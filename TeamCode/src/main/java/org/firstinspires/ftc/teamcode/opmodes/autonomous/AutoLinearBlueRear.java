package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.subsystems.ArmSubSystem;
import org.firstinspires.ftc.teamcode.subsystems.ImuDevice;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveByGyro;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveParameters;
import org.firstinspires.ftc.teamcode.util.RobotDevices;

@Autonomous(
        name = "Auto - Blue Rear"
)
public class AutoLinearBlueRear extends LinearOpMode {


    protected RobotDevices robotDevices;

    protected MecanumDriveByGyro _move;
    protected ArmSubSystem _armSubSystem;

    protected DistanceSensor frontSensor, rearSensor;

    protected Servo pixelHold0,pixelHold1;

    protected Servo [] sensorServos;


    protected final double LEFT=-90;
    protected final double RIGHT=90;
    protected static final int LEFT_SIDE=1;
    protected static final int RIGHT_SIDE=2;

    public double turnDirection() {
        if (left)
            return this.LEFT;
        else
            return this.RIGHT;
    }

    public int side() {
        if (left)
            return LEFT_SIDE;
        else
            return RIGHT_SIDE;
    };

    boolean blue = false;
    boolean left = false;



    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.setAutoClear(false);

        robotDevices = RobotDevices.getDevices(hardwareMap);


        // set up MovementThread

        //colorSensorDeviceLeft = new ColorSensorDeviceImpl(robotDevices.colorSensorLeft);
        //colorSensorDeviceRight = new ColorSensorDeviceImpl(robotDevices.colorSensorRight);

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



        waitForStart();


        // Get to position and drop first pixel
        // TODO: fix servo positions
        for (Servo sensorservo: sensorServos) {
            sensorservo.setPosition(1.0);
        }
        driveRight(30);
        if (frontSensor.getDistance(DistanceUnit.MM) <=25) {
            //placefront
            driveForward(15);
            dropPixel();
        }
        else if (rearSensor.getDistance(DistanceUnit.MM)<=25) {
            //placerear
            driveReverse(6);
            dropPixel();
            driveForward(21);
        }
        else {
            //place center
            driveRight(6);
            dropPixel();
            driveLeft(6);
            driveForward(15);
        }
        // TODO: fix servo positions
        for (Servo sensorservo: sensorServos) {
            sensorservo.setPosition(0.0);
        }



        //now place next pixel
        driveForward(84);

        //deliver pixel!
        _armSubSystem.raisePixelHold();
        _armSubSystem.runBeltInches(-1);
        _armSubSystem.runBeltInches(20);
        _armSubSystem.lowerPixelHold();




    }

    private void dropPixel()  throws InterruptedException {
        // TODO:
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
}