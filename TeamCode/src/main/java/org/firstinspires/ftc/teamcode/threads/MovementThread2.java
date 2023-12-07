package org.firstinspires.ftc.teamcode.threads;

import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.PixelDelivery;
import org.firstinspires.ftc.teamcode.subsystems.PlaneLauncher;
import org.firstinspires.ftc.teamcode.util.Constants;
import org.firstinspires.ftc.teamcode.util.Debounce;
import org.firstinspires.ftc.teamcode.util.MotorRampProfile;


public class MovementThread2 extends RobotThread {

    private Gamepad _gamepad;
    private MecanumDrive _drive;
    private PlaneLauncher _launch;
    private Servo[] _sensorServos;
    private DistanceSensor _wallSensor;
    private Telemetry _telemetry;
    private PixelDelivery _pixelDelivery;

    MotorRampProfile _Joy1X, _Joy1Y, _Joy2X;
    Telemetry.Item T_wall;

    public MovementThread2(Gamepad gamepad,
                           MecanumDrive drive,
                           PlaneLauncher launch,
                           Servo[] sensorServos,
                           DistanceSensor wallSensor,
                           PixelDelivery pixelDelivery,
                           Telemetry telemetry) {
        this._gamepad = gamepad;
        this._drive = drive;
        this._launch = launch;
        this._sensorServos = sensorServos;
        this._wallSensor = wallSensor;
        this._pixelDelivery = pixelDelivery;
        this._telemetry = telemetry;

        _Joy1Y = new MotorRampProfile(Constants.MecanumDrive.RAMP_RATE_J1X);
        _Joy1X = new MotorRampProfile(Constants.MecanumDrive.RAMP_RATE_J1Y);
        _Joy2X = new MotorRampProfile(Constants.MecanumDrive.RAMP_RATE_J2X);

        T_wall = _telemetry.addData("Wall sensor", wallSensor.getDistance(DistanceUnit.MM));
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
        boolean drop = true;
        Debounce buttona = new Debounce(1000);
        while (!isCancelled()) {

            if (_gamepad.x && _gamepad.left_bumper) {
                _launch.launch();
            }

            Thread deploy;
            if (buttona.checkPress(_gamepad.a)) {

                deploy = new Thread() {
                    @Override
                    public void run() {
                        _pixelDelivery.leftPixelDrop();
                        _pixelDelivery.rightPixelDrop();
                        try {
                            Thread.sleep(1000);
                            _pixelDelivery.rightPixelReset();
                            _pixelDelivery.leftPixelReset();

                        } catch (InterruptedException ignored) {

                        }

                    }
                };
                deploy.start();
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


