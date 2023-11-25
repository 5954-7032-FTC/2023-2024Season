package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.subsystems.ArmSubSystem;
import org.firstinspires.ftc.teamcode.subsystems.ImuDevice;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveByGyro;
import org.firstinspires.ftc.teamcode.subsystems.PixelDelivery;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveImpl;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveParameters;
import org.firstinspires.ftc.teamcode.util.Constants;
import org.firstinspires.ftc.teamcode.util.RobotDevices;
import org.firstinspires.ftc.teamcode.util.TweakableDouble;
import org.firstinspires.ftc.teamcode.util.motorRampProfile;

@TeleOp(name = "MoveTest")
public class MoveTest extends LinearOpMode {

    protected Telemetry.Item T_sensorServos;
//    protected Servo leftPixelArm,leftPixelFlip,rightPixelArm,rightPixelFlip;
//    protected Servo [] sensorServos;
//    protected DistanceSensor frontSensor, rearSensor;
    protected PixelDelivery autoPixelDelivery;
    @Override
    public void runOpMode() throws InterruptedException {
        RobotDevices robotDevices=RobotDevices.getDevices(hardwareMap);

        MecanumDriveParameters driveParameters = new MecanumDriveParameters();
        driveParameters.telemetry = telemetry;
        driveParameters.motors = robotDevices.wheels;
        driveParameters.ENCODER_WHEELS = new int[]{0,1,2,3};
        driveParameters.FREE_WHEELS = new int[]{0, 1, 2, 3};
        driveParameters.REVERSED_WHEELS = new int[]{2, 3};
        MecanumDriveByGyro drive = new MecanumDriveByGyro(driveParameters, new ImuDevice(robotDevices.imunew));

        IMU imu  = robotDevices.imunew;

        ArmSubSystem _armSubSystem = new ArmSubSystem(
                robotDevices.intakeServos,
                robotDevices.lowBelt,
                robotDevices.highBelt,
                robotDevices.armLift,
                robotDevices.upperArmLimit,
                robotDevices.lowerArmLimit,
                robotDevices.pixelHold);
        //sensorServos = robotDevices.sensorServos;

        T_sensorServos = telemetry.addData("SensorServos", "0=(%f),1=(%f)",0.0,0.0);

        motorRampProfile _Joy1X, _Joy1Y, _Joy2X;
        TweakableDouble _ramp_rate_J1X =  new TweakableDouble("RampRateJ1X", 0.02, 1.5);
        TweakableDouble _ramp_rate_J1Y =  new TweakableDouble("RampRateJ1Y", 0.02, 1.5);
        TweakableDouble _ramp_rate_J2X =  new TweakableDouble("RampRateJ2X", 0.02, 1.5);


        _Joy1Y = new motorRampProfile(_ramp_rate_J1X.value);
        _Joy1X = new motorRampProfile(_ramp_rate_J1Y.value);
        _Joy2X = new motorRampProfile(_ramp_rate_J2X.value);

        //Servo pixelHold0,pixelHold1;



        autoPixelDelivery = new PixelDelivery(telemetry,
                robotDevices.leftPixelArm,
                robotDevices.leftPixelFlip,
                robotDevices.rightPixelArm,
                robotDevices.rightPixelFlip,
                robotDevices.sensorServos,
                robotDevices.frontSensor,
                robotDevices.rearSensor);
//        Servo [] sensorServos;
//
//        //pixelHold0 = robotDevices.pixelFloor0;
//        //pixelHold1 = robotDevices.pixelFloor1;
//
//        leftPixelArm = robotDevices.leftPixelArm;
//        rightPixelArm = robotDevices.rightPixelArm;
//        leftPixelFlip = robotDevices.leftPixelFlip;
//        rightPixelFlip = robotDevices.rightPixelFlip;
//
//        sensorServos = robotDevices.sensorServos;

        Telemetry.Item T_pixelHold = telemetry.addData("PixelHold","0=(%f),1=(%f)",0.0,0.0);
        Telemetry.Item T_sensorServos = telemetry.addData("SensorServos", "0=(%f),1=(%f)",0.0,0.0);
        Telemetry.Item T_sensors = telemetry.addData("Sensors:", "Front:(%f) Rear(%f)",0.0,0.0);

        YawPitchRollAngles robotOrientation;
        robotOrientation = imu.getRobotYawPitchRollAngles();

        Telemetry.Item T_IMU = telemetry.addData("IMU", "(Yaw:(%f), Pitch:(%f), Roll:(%f)",
                robotOrientation.getYaw(AngleUnit.DEGREES),
                robotOrientation.getPitch(AngleUnit.DEGREES),
                robotOrientation.getRoll(AngleUnit.DEGREES)
        );
        waitForStart();
        // Reset Yaw
        imu.resetYaw();

        //frontSensor = robotDevices.frontSensor;
        //rearSensor = robotDevices.rearSensor;



        while(opModeIsActive()) {
            telemetry.update();


            robotOrientation = imu.getRobotYawPitchRollAngles();
            T_IMU.setValue("(Yaw:(%f), Pitch:(%f), Roll:(%f)",
                    robotOrientation.getYaw(AngleUnit.DEGREES),
                    robotOrientation.getPitch(AngleUnit.DEGREES),
                    robotOrientation.getRoll(AngleUnit.DEGREES)
            );
            telemetry.update();

            if (gamepad1.dpad_left) {
                drive.driveLeft(30*Constants.X_DISTANCE_RATIO);
            }
            else if (gamepad1.dpad_right) {
                drive.driveRight(30*Constants.X_DISTANCE_RATIO);
            }
            else if (gamepad1.dpad_down) {
                drive.driveForward(-12*Constants.Y_DISTANCE_RATIO);
            }
            else if (gamepad1.dpad_up) {
                drive.driveForward(12*Constants.Y_DISTANCE_RATIO);
            }

            if (gamepad1.a) {
                autoPixelDelivery.extendSensors();
            }
            else {
                autoPixelDelivery.retractSensors();
            }

            if (gamepad1.b) {
                _armSubSystem.moveArmMillis(true,250);
                _armSubSystem.moveArmMillis(false, 250);
            }


            T_sensors.setValue("Front:(%f) Rear(%f)",autoPixelDelivery.readFrontSensor(),autoPixelDelivery.readRearSensor());

        }


        double fine_control = gamepad1.right_trigger>=0.2?1.2-gamepad1.right_trigger:1.0;
        drive.moveRect(
                _Joy1Y.ramp(deadzone(gamepad1.left_stick_y* fine_control, Constants.MecanumDrive.ZONE_FORWARD)),
                _Joy1X.ramp(deadzone(gamepad1.left_stick_x* fine_control, Constants.MecanumDrive.ZONE_LATERAL)),
                _Joy2X.ramp(deadzone(gamepad1.right_stick_x* fine_control, Constants.MecanumDrive.ZONE_ROTATION))
        );

    }

//    public boolean testFrontSensor(int mm) {
//        return frontSensor.getDistance(DistanceUnit.MM) <=mm;
//    }
//
//    public boolean testRearSensor(int mm) {
//        return rearSensor.getDistance(DistanceUnit.MM) <=mm;
//    }
//    public void extendSensors() {
//        sensorServos[0].setPosition(0.65); // front
//        sensorServos[1].setPosition(0.4);
//        T_sensorServos.setValue("0=(%f),1=(%f)",0.7,0.55);
//    }
//
//    public void retractSensors() {
//        sensorServos[0].setPosition(0.0);
//        sensorServos[1].setPosition(1.0);
//        T_sensorServos.setValue("0=(%f),1=(%f)",0.0,1.0);
//    }
//
//    private void leftPixelDrop( Constants.pixelDropPositions position) throws InterruptedException {
//        if ( position == Constants.pixelDropPositions.LEFT_RESET) {
//            leftPixelFlip.setPosition(0.0);
//            Thread.sleep(position.ms);
//            leftPixelArm.setPosition(position.pos);
//            return;
//        }
//        leftPixelArm.setPosition(position.pos);
//        Thread.sleep(position.ms);
//        leftPixelFlip.setPosition(1.0);
//    }
//
//    private void rightPixelDrop( Constants.pixelDropPositions position) throws InterruptedException {
//        if ( position == Constants.pixelDropPositions.RIGHT_RESET) {
//            rightPixelFlip.setPosition(0.0);
//            Thread.sleep(position.ms);
//            rightPixelArm.setPosition(position.pos);
//            return;
//        }
//        rightPixelArm.setPosition(position.pos);
//        Thread.sleep(position.ms);
//        rightPixelFlip.setPosition(1.0);
//    }

    private double deadzone(double power, double zone)  {
        return Math.abs(power) > zone ? power : 0.0 ;
    }
}

