package org.firstinspires.ftc.teamcode.threads;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveImpl;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveParameters;
import org.firstinspires.ftc.teamcode.subsystems.PixelDelivery;
import org.firstinspires.ftc.teamcode.subsystems.PlaneLauncher;
import org.firstinspires.ftc.teamcode.util.Constants;
import org.firstinspires.ftc.teamcode.util.Debounce;
import org.firstinspires.ftc.teamcode.util.MotorRampProfile;


public class MovementThread extends RobotThread {

    private Gamepad _gamepad;

    private MecanumDrive _drive;

    MotorRampProfile _Joy1X, _Joy1Y, _Joy2X;
    PlaneLauncher _launch;

    Servo[] _sensorServos;

    DistanceSensor _wallSensor;
    Telemetry _telemetry;
    Telemetry.Item T_wall;


    PixelDelivery _pixelDelivery;

    public MovementThread(Gamepad gamepad, DcMotor[] motors, Telemetry telemetry, IMU imu, Servo launchServo, Servo[] sensorServos, DistanceSensor wallSesor, PixelDelivery pixelDelivery) {

        this.init(gamepad, motors, telemetry, imu, launchServo, sensorServos, wallSesor, pixelDelivery);
    }

    protected void init(Gamepad gamepad, DcMotor[] motors, Telemetry telemetry, IMU imu, Servo launchServo, Servo[] sensorServos, DistanceSensor wallSensor, PixelDelivery pixelDelivery) {
        this._gamepad = gamepad;

        this._pixelDelivery = pixelDelivery;
        MecanumDriveParameters driveParameters = new MecanumDriveParameters();
        driveParameters.telemetry = telemetry;
        driveParameters.motors = motors;
        driveParameters.ENCODER_WHEELS = new int[]{0, 1, 2, 3};
        driveParameters.FREE_WHEELS = new int[]{0, 1, 2, 3};
        driveParameters.REVERSED_WHEELS = new int[]{2, 3};
        driveParameters.imu = imu;
        _drive = new MecanumDriveImpl(driveParameters);

        _Joy1Y = new MotorRampProfile(Constants.MecanumDrive.RAMP_RATE_J1X);
        _Joy1X = new MotorRampProfile(Constants.MecanumDrive.RAMP_RATE_J1Y);
        _Joy2X = new MotorRampProfile(Constants.MecanumDrive.RAMP_RATE_J2X);


        _launch = new PlaneLauncher(launchServo);

        _sensorServos = sensorServos;

        _wallSensor = wallSensor;
        _telemetry = telemetry;
        T_wall = _telemetry.addData("Wall sensor", _wallSensor.getDistance(DistanceUnit.MM));


    }

    private double deadzone(double power, double zone) {
        return Math.abs(power) > zone ? power : 0.0;
    }


    double fine_control = 1.0;

    public void run() {
        _sensorServos[0].setPosition(Servo.MIN_POSITION);
        _sensorServos[1].setPosition(Servo.MAX_POSITION);
        double forward;
        _launch.reset();
        boolean drop=true;
        Debounce buttona = new Debounce(250);
        while (!isCancelled()) {

            if (_gamepad.x && _gamepad.left_bumper) {
                _launch.launch();
            }

            if (buttona.checkPress(_gamepad.a)) {

                if (drop) {
                    _pixelDelivery.leftPixelDrop();
                    _pixelDelivery.rightPixelDrop();
                    drop = false;
                } else {
                    drop = true;
                    try {
                        _pixelDelivery.rightPixelReset();
                        _pixelDelivery.leftPixelReset();
                    } catch (InterruptedException ignored) {
                    }
                }
            }



            if (_gamepad.y) {
                _launch.reset();
            }

            T_wall.setValue(_wallSensor.getDistance(DistanceUnit.MM));
            _telemetry.update();

            fine_control = _gamepad.right_trigger >= 0.3 ? 1.3 - _gamepad.right_trigger : 1.0;
            _drive.moveRect(
                    _Joy1Y.ramp(deadzone(_gamepad.left_stick_y * fine_control, Constants.MecanumDrive.ZONE_FORWARD)),
                    _Joy1X.ramp(deadzone(_gamepad.left_stick_x * fine_control, Constants.MecanumDrive.ZONE_LATERAL)),
                    _Joy2X.ramp(deadzone(_gamepad.right_stick_x * fine_control, Constants.MecanumDrive.ZONE_ROTATION))
            );
        }
    }
}


