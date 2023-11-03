package org.firstinspires.ftc.teamcode.threads;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveImpl;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveParameters;
import org.firstinspires.ftc.teamcode.util.Constants;
import org.firstinspires.ftc.teamcode.util.TweakableDouble;
import org.firstinspires.ftc.teamcode.util.motorRampProfile;


public class MovementThread extends RobotThread {

    private Gamepad _gamepad;

    private MecanumDrive drive;

    motorRampProfile _Joy1X, _Joy1Y, _Joy2X;



    public MovementThread(Gamepad gamepad, DcMotor[] motors, Telemetry telemetry) {

        this.init(gamepad, motors, telemetry);
    }

    protected void init(Gamepad gamepad, DcMotor[] motors, Telemetry telemetry) {
        this._gamepad = gamepad;

        MecanumDriveParameters driveParameters = new MecanumDriveParameters();
        driveParameters.telemetry = telemetry;
        driveParameters.motors = motors;
        driveParameters.ENCODER_WHEELS = new int[]{0,1,2,3};
        driveParameters.FREE_WHEELS = new int[]{0, 1, 2, 3};
        driveParameters.REVERSED_WHEELS = new int[]{2, 3};
        drive = new MecanumDriveImpl(driveParameters);

        _Joy1Y = new motorRampProfile(Constants.MecanumDrive.RAMP_RATE_J1X);
        _Joy1X = new motorRampProfile(Constants.MecanumDrive.RAMP_RATE_J1Y);
        _Joy2X = new motorRampProfile(Constants.MecanumDrive.RAMP_RATE_J2X);

    }



    private double deadzone(double power, double zone)  {
        return Math.abs(power) > zone ? power : 0.0 ;
    }

    public void run() {
        while (!isCancelled()) {
            //if (includeTweaks) checkTweaks();
            if (_gamepad.a) {
                drive.setMotorSpeeds(new double[] {0.5,0.5,0.5,0.5});
                continue;
            }
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
