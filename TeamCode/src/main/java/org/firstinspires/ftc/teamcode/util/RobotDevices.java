package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.teamcode.subsystems.DistanceSensorDevice;
/**
 * RobotDevices is a singleton that holds references for all hardware.&nbsp;
* @author      Greg Weaver
* @version     %I%, %G%
* @since       1.0
*/
public class RobotDevices {

    //public ColorSensor colorSensorLeft,colorSensorRight;
    public DcMotor [] wheels;
    public DcMotor [] armLift;
    public DcMotor lowBelt;
    public DcMotor highBelt;
    public BNO055IMU imu;


    //public ColorSensor colorSensorFront, colorSensorBack;
    public TouchSensor upperArmLimit, lowerArmLimit;
    public CRServo[] intakeServos;
    public Servo[] pixelFloor;
    public Servo droneRelease;
    public Servo[] sensorServos;
    public DistanceSensor frontSensor, rearSensor;
    public Servo pixelHold0, pixelHold1;


    //public ArmReleaseImpl arm_release;


    // public ColorSensor cone_detector;
    //public Servo[] lift_servos;
    //public DcMotor lift_motor;
    //public TouchSensor bottom_stop;
    //public DistanceSensorDevice post_sensor;//,bottom_cone;

    protected static RobotDevices robotDevices;

    public static RobotDevices getDevices(HardwareMap hardwareMap) {
        if (robotDevices == null ) {
            robotDevices = new RobotDevices(hardwareMap);
        }
        return robotDevices;
    }

    private RobotDevices(HardwareMap hardwareMap) {
        //colorSensorRight = hardwareMap.colorSensor.get("RIGHT_COLOR");
        //colorSensorLeft = hardwareMap.colorSensor.get("LEFT_COLOR");
        wheels = new DcMotor[]{
                hardwareMap.dcMotor.get("D_MOT_FR"),
                hardwareMap.dcMotor.get("D_MOT_RR"),
                hardwareMap.dcMotor.get("D_MOT_RL"),
                hardwareMap.dcMotor.get("D_MOT_FL")
        };

        imu = hardwareMap.get(BNO055IMU.class, "imu");



        armLift = new DcMotor[] {
                hardwareMap.dcMotor.get("ARM_MOT0"),
                hardwareMap.dcMotor.get("ARM_MOT1")
        };
        lowBelt =  hardwareMap.dcMotor.get("BELT_MOT_LOW");
        highBelt = hardwareMap.dcMotor.get("BELT_MOT_HIGH");


        //colorSensorFront = hardwareMap.colorSensor.get("FRONT_COLOR");
        //colorSensorBack = hardwareMap.colorSensor.get("BACK_COLOR");

        upperArmLimit = hardwareMap.touchSensor.get("ARM_UPPER_LIMIT");
        lowerArmLimit = hardwareMap.touchSensor.get("ARM_LOWER_LIMIT");
        intakeServos = new CRServo[] {
                hardwareMap.crservo.get("INTAKE0"),
                hardwareMap.crservo.get("INTAKE1")
        };
        droneRelease = hardwareMap.servo.get("DRONE_RELEASE");

        pixelFloor = new Servo[]{
                hardwareMap.servo.get("PIXEL_HOLD0"),
                hardwareMap.servo.get("PIXEL_HOLD1")
        };

        sensorServos = new Servo[] {

                hardwareMap.servo.get("FRONT_SENSOR_SERVO"),
                hardwareMap.servo.get("REAR_SENSOR_SERVO")

        };
        frontSensor =hardwareMap.get(DistanceSensor.class,"FRONT_SENSOR");
        rearSensor = hardwareMap.get(DistanceSensor.class,"REAR_SENSOR");

        pixelHold0 = hardwareMap.servo.get("PIXEL_HOLD0");
        pixelHold1 = hardwareMap.servo.get("PIXEL_HOLD1");
        droneRelease = hardwareMap.servo.get("DRONE_RELEASE");

            /*
    ** Arm consists of several parts
    * 1. Floor suck
        intakeServos = new Servo[] {
                hardwareMap.servo.get("INTAKE0"),
                hardwareMap.servo.get("INTAKE1")
        };

    * 2. Lower Conveyor (lowBelt)
        lowBelt =  hardwareMap.dcMotor.get("BELT_MOT_LOW");

    * 3. Upper Conveyor (highBelt)
        highBelt = hardwareMap.dcMotor.get("BELT_MOT_HIGH");

    * 4. Upper Conveyor bend (armLift, upperArmLimit, lowerArmLimit)
        armLift = new DcMotor[] {
                hardwareMap.dcMotor.get("ARM_MOT0"),
                hardwareMap.dcMotor.get("ARM_MOT1")
        };
        upperArmLimit = hardwareMap.touchSensor.get("ARM_UPPER_LIMIT");
        lowerArmLimit = hardwareMap.touchSensor.get("ARM_LOWER_LIMIT");

     1-4 done

    * 5. Color Sensor for auto
        colorSensorFront = hardwareMap.colorSensor.get("FRONT_COLOR");
        colorSensorBack = hardwareMap.colorSensor.get("BACK_COLOR");


    *  6 drone release
        droneRelease = hardwareMap.servo.get("DRONE_RELEASE");



*  7 pixel hold detection and tooling
        pixelFloor = hardwareMap.servo.get("PIXEL_SERVO");
        pixelHold0 = hardwareMap.servo.get("PIXEL_HOLD0");
        pixelHold1 = hardwareMap.servo.get("PIXEL_HOLD1");

*
        frontSensor = new DistanceSensorDevice(hardwareMap.get(DistanceSensor.class, "FRONT_SENSOR"));
        rearSensor = new DistanceSensorDevice(hardwareMap.get(DistanceSensor.class, "REAR_SENSOR"));
        *
     */
    }
}
