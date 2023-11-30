package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

/**
 * RobotDevices is a singleton that holds references for all hardware.&nbsp;
* @author      Greg Weaver
* @version     %I%, %G%
* @since       1.0
*/
public class RobotDevices {

    public DcMotor [] wheels;
    public DcMotor [] armLift;
    public DcMotor lowBelt;
    public DcMotor highBelt;
    //public BNO055IMU imu;
    public IMU imunew;
    public TouchSensor upperArmLimit, lowerArmLimit;
    public CRServo[] intakeServos;
    public Servo[] pixelHold;
    public Servo droneRelease;
    public Servo[] sensorServos;
    public DistanceSensor frontSensor, rearSensor;
    //public Servo pixelFloor0, pixelFloor1;
    public DistanceSensor wallSensor;

    public Servo leftPixelArm, rightPixelArm,leftPixelFlip,rightPixelFlip;


    protected static RobotDevices robotDevices;

    public static RobotDevices getDevices(HardwareMap hardwareMap) {
        if (robotDevices == null ) {
            robotDevices = new RobotDevices(hardwareMap);
        }
        return robotDevices;
    }

    private RobotDevices(HardwareMap hardwareMap) {
        wheels = new DcMotor[]{
                hardwareMap.dcMotor.get("D_MOT_FR"),
                hardwareMap.dcMotor.get("D_MOT_RR"),
                hardwareMap.dcMotor.get("D_MOT_RL"),
                hardwareMap.dcMotor.get("D_MOT_FL")
        };

        //imu = hardwareMap.get(BNO055IMU.class, "imu");

        imunew = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters imuParameters;
        imuParameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                        RevHubOrientationOnRobot.UsbFacingDirection.UP
                )
        );
        imunew.initialize(imuParameters);


        armLift = new DcMotor[] {
                hardwareMap.dcMotor.get("ARM_MOT0"),
                hardwareMap.dcMotor.get("ARM_MOT1")
        };
        lowBelt =  hardwareMap.dcMotor.get("BELT_MOT_LOW");
        highBelt = hardwareMap.dcMotor.get("BELT_MOT_HIGH");

        upperArmLimit = hardwareMap.touchSensor.get("ARM_UPPER_LIMIT");
        lowerArmLimit = hardwareMap.touchSensor.get("ARM_LOWER_LIMIT");
        intakeServos = new CRServo[] {
                hardwareMap.crservo.get("INTAKE0"),
                hardwareMap.crservo.get("INTAKE1")
        };
        droneRelease = hardwareMap.servo.get("DRONE_RELEASE");

        pixelHold = new Servo[]{
                hardwareMap.servo.get("PIXEL_HOLD0"),
                hardwareMap.servo.get("PIXEL_HOLD1")
        };

        sensorServos = new Servo[] {

                hardwareMap.servo.get("FRONT_SENSOR_SERVO"),
                hardwareMap.servo.get("REAR_SENSOR_SERVO")

        };
        frontSensor =hardwareMap.get(DistanceSensor.class,"FRONT_SENSOR");
        rearSensor = hardwareMap.get(DistanceSensor.class,"REAR_SENSOR");


        leftPixelArm = hardwareMap.servo.get("LEFT_PIXEL_ARM");
        rightPixelArm = hardwareMap.servo.get("RIGHT_PIXEL_ARM");
        leftPixelFlip = hardwareMap.servo.get("LEFT_PIXEL_FLIP");
        rightPixelFlip = hardwareMap.servo.get("RIGHT_PIXEL_FLIP");


        droneRelease = hardwareMap.servo.get("DRONE_RELEASE");

        wallSensor = hardwareMap.get(DistanceSensor.class,"WALL_SENSOR");
    }
}
