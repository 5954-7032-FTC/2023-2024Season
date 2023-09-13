package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.teamcode.subsystems.ArmReleaseImpl;
import org.firstinspires.ftc.teamcode.subsystems.DistanceSensorDevice;
/**
 * RobotDevices is a singleton that holds references for all hardware.&nbsp;
* @author      Greg Weaver
* @version     %I%, %G%
* @since       1.0
*/
public class RobotDevices {

    public ColorSensor colorSensorLeft,colorSensorRight;
    public DcMotor [] wheels;
    public Servo[] lift_servos;
    public BNO055IMU imu;
    public DcMotor lift_motor;
    public TouchSensor bottom_stop;
    public DistanceSensorDevice post_sensor;//,bottom_cone;

    public ArmReleaseImpl arm_release;


   // public ColorSensor cone_detector;

    protected static RobotDevices robotDevices;

    public static RobotDevices getDevices(HardwareMap hardwareMap) {
        if (robotDevices == null ) {
            robotDevices = new RobotDevices(hardwareMap);
        }
        return robotDevices;
    }

    private RobotDevices(HardwareMap hardwareMap) {
        colorSensorRight = hardwareMap.colorSensor.get("RIGHT_COLOR");
        colorSensorLeft = hardwareMap.colorSensor.get("LEFT_COLOR");
        wheels = new DcMotor[]{
                hardwareMap.dcMotor.get("D_FR"),
                hardwareMap.dcMotor.get("D_RR"),
                hardwareMap.dcMotor.get("D_RL"),
                hardwareMap.dcMotor.get("D_FL")
        };
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        lift_motor = hardwareMap.dcMotor.get("LIFT");
        lift_servos = new Servo[]{
                hardwareMap.servo.get("CLAW0"),
                hardwareMap.servo.get("CLAW1")
        };
        bottom_stop = hardwareMap.touchSensor.get("BSTOP");
        post_sensor = new DistanceSensorDevice(hardwareMap.get(DistanceSensor.class, "C_STOP"));
        arm_release = new ArmReleaseImpl(hardwareMap.servo.get("ARM_RELEASE"));
    }
}
