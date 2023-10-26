package org.firstinspires.ftc.teamcode.threads;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.ArmSubSystem;
import org.firstinspires.ftc.teamcode.util.Constants;
import org.firstinspires.ftc.teamcode.util.Debounce;
import org.firstinspires.ftc.teamcode.util.Tweakable;
import org.firstinspires.ftc.teamcode.util.motorRampProfile;

public class ArmControlThread extends RobotThread {

    private motorRampProfile  _Joy1Y, _Joy2Y;
    private Telemetry _telemetry;
    private ArmSubSystem _armSubSystem;
    private Gamepad _gamepad;

    private Debounce _buttonA;

    public ArmControlThread(Gamepad gamepad, Telemetry telemetry,
                            CRServo[] intake_servos, DcMotor lowerBeltMotor, DcMotor upperBeltMotor, DcMotor[] bendMotors, TouchSensor upperArmLimit, TouchSensor lowerArmLimit) {
        _armSubSystem = new ArmSubSystem(intake_servos,lowerBeltMotor,upperBeltMotor,bendMotors,upperArmLimit,lowerArmLimit);


        //TODO set ramp rate in constants
        _Joy1Y = new motorRampProfile(1.5);
        _Joy2Y = new motorRampProfile(1.5);

        _gamepad = gamepad;

        _telemetry = telemetry;

        _buttonA = new Debounce(150);

    }

    @Override
    public void run() {
        while (!isCancelled()) {
            // right joystick up/down controls the bend amount

            // TODO set dead zone
            // if right stick is pushed more than 20% and neither limit switch is triggered, apply power, else 0.
            if ( Math.abs(_gamepad.right_stick_y)  > Constants.armControlDeadzone &&  ! ( _armSubSystem.lowerArmLimit() || _armSubSystem.upperArmLimit() ) ) {
                    _armSubSystem.moveArm(_Joy2Y.ramp(_gamepad.right_stick_y));
            }
            else {
                _armSubSystem.moveArm(0);
            }

            // TODO set dead zone
            if (_gamepad.right_trigger>Constants.armControlDeadzone) {
                _armSubSystem.intakeForward();
                _armSubSystem.beltPower(0.5);
            }

            // TODO set dead zone
            if (_gamepad.left_trigger>Constants.armControlDeadzone) {
                _armSubSystem.intakeReverse();
                _armSubSystem.beltPower(-0.5);
            }

            /* TODO set dead zone
            if (_gamepad.left_stick_y > Constants.armControlDeadzone) {
                _armSubSystem.beltPower(_Joy1Y.ramp(_gamepad.left_stick_y));
            }
            */


        }

    }
}
