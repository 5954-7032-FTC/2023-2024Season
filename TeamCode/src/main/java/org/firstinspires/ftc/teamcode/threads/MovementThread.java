package org.firstinspires.ftc.teamcode.threads;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveGyroImpl;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveImpl;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveParameters;
import org.firstinspires.ftc.teamcode.subsystems.PlaneLauncher;
import org.firstinspires.ftc.teamcode.util.Constants;
import org.firstinspires.ftc.teamcode.util.RobotDevices;
import org.firstinspires.ftc.teamcode.util.TweakableDouble;
import org.firstinspires.ftc.teamcode.util.motorRampProfile;


public class MovementThread extends RobotThread {

    private Gamepad _gamepad;

    private MecanumDrive drive;

    motorRampProfile _Joy1X, _Joy1Y, _Joy2X;
    PlaneLauncher _launch;

    Servo [] _sensorServos;




    public MovementThread(Gamepad gamepad, DcMotor[] motors, Telemetry telemetry,BNO055IMU imu, Servo launchServo, Servo [] sensorServos) {

        this.init(gamepad, motors, telemetry,imu,launchServo, sensorServos);
    }

    protected void init(Gamepad gamepad, DcMotor[] motors, Telemetry telemetry, BNO055IMU imu,Servo launchServo, Servo [] sensorServos) {
        this._gamepad = gamepad;

        MecanumDriveParameters driveParameters = new MecanumDriveParameters();
        driveParameters.telemetry = telemetry;
        driveParameters.motors = motors;
        driveParameters.ENCODER_WHEELS = new int[]{0,1,2,3};
        driveParameters.FREE_WHEELS = new int[]{0, 1, 2, 3};
        driveParameters.REVERSED_WHEELS = new int[]{2, 3};
        driveParameters.imu = imu;
        drive = new MecanumDriveImpl(driveParameters);

        _Joy1Y = new motorRampProfile(Constants.MecanumDrive.RAMP_RATE_J1X);
        _Joy1X = new motorRampProfile(Constants.MecanumDrive.RAMP_RATE_J1Y);
        _Joy2X = new motorRampProfile(Constants.MecanumDrive.RAMP_RATE_J2X);


        _launch = new PlaneLauncher(launchServo);

        _sensorServos = sensorServos;
    }

    private double deadzone(double power, double zone)  {
        return Math.abs(power) > zone ? power : 0.0 ;
    }

    public void run() {
        _sensorServos[0].setPosition(Servo.MIN_POSITION);
        _sensorServos[1].setPosition(Servo.MAX_POSITION);
        while (!isCancelled()) {
            if (_gamepad.x && _gamepad.left_trigger > 0.1) {
                _launch.launch();
            }
            if (_gamepad.y) {
                _launch.reset();
            }
            //if (includeTweaks) checkTweaks();
            if (_gamepad.right_trigger > 0.2) {
                drive.moveRect(
                        _Joy1Y.ramp(deadzone(_gamepad.left_stick_y* Constants.MecanumDrive.FINE_CONTROL, Constants.MecanumDrive.ZONE_FORWARD)),
                        _Joy1X.ramp(deadzone(_gamepad.left_stick_x* Constants.MecanumDrive.FINE_CONTROL, Constants.MecanumDrive.ZONE_LATERAL)),
                        _Joy2X.ramp(deadzone(_gamepad.right_stick_x* Constants.MecanumDrive.FINE_CONTROL, Constants.MecanumDrive.ZONE_ROTATION))
                );
            }
            else {
                drive.moveRect(
                        _Joy1Y.ramp(deadzone(_gamepad.left_stick_y, Constants.MecanumDrive.ZONE_FORWARD)),
                        _Joy1X.ramp(deadzone(_gamepad.left_stick_x, Constants.MecanumDrive.ZONE_LATERAL)),
                        _Joy2X.ramp(deadzone(_gamepad.right_stick_x, Constants.MecanumDrive.ZONE_ROTATION))
                );
            }

        }
    }


}
