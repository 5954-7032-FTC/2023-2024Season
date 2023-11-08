package org.firstinspires.ftc.teamcode.threads;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.ArmSubSystem;
import org.firstinspires.ftc.teamcode.util.Constants;
import org.firstinspires.ftc.teamcode.util.Debounce;
import org.firstinspires.ftc.teamcode.util.motorRampProfile;

public class ArmControlThread extends RobotThread {

    private motorRampProfile  _Joy1Y, _Joy2Y;
    private Telemetry _telemetry;
    private ArmSubSystem _armSubSystem;
    private Gamepad _gamepad;

    private Debounce _buttonA;

    private Telemetry.Item _T_Intake_Speed;

    public ArmControlThread(Gamepad gamepad, Telemetry telemetry,
                            CRServo[] intake_servos, DcMotor lowerBeltMotor, DcMotor upperBeltMotor, DcMotor[] bendMotors, TouchSensor upperArmLimit, TouchSensor lowerArmLimit, Servo[] pixelHold) {
        _armSubSystem = new ArmSubSystem(intake_servos,lowerBeltMotor,upperBeltMotor,bendMotors,upperArmLimit,lowerArmLimit,pixelHold);


        //TODO set ramp rate in constants
        _Joy1Y = new motorRampProfile(1.5);
        _Joy2Y = new motorRampProfile(1.5);

        _gamepad = gamepad;

        _telemetry = telemetry;

        _T_Intake_Speed = _telemetry.addData("IntakeSpeed",0.0);

        _buttonA = new Debounce(150);

    }

    @Override
    public void run() {
        while (!isCancelled()) {
            // right joystick up/down controls the bend amount

            // TODO set dead zone
            /* Logic for allowing the arm to move:
            ** stick must be larger than the deadzone
            * OR
            ** if upper arm limit is triggered AND stick is negative, move
            ** if lower arm limit is trriggered AND stick is positive, move
            */
            if ( Math.abs(_gamepad.right_stick_y)  > Constants.armControlDeadzone
                    && ! (
                         _armSubSystem.lowerArmLimit() || _armSubSystem.upperArmLimit()
                    )
                ||
                    (
                    (_gamepad.right_stick_y < 0 && _armSubSystem.upperArmLimit())
                    ||
                    (_gamepad.right_stick_y > 0 && _armSubSystem.lowerArmLimit())
                    )
            ) {
                    _armSubSystem.moveArm(_Joy2Y.ramp(_gamepad.right_stick_y));
            }
            else {
                _armSubSystem.moveArm(0);
            }

            if (_gamepad.a) {
                _armSubSystem.raisePixelHold();
                _armSubSystem.intakeForward();
                _armSubSystem.beltPower(-1.0,_armSubSystem.lowerArmLimit());
                continue;
            }
            else {
                _armSubSystem.lowerPixelHold();
                // TODO set dead zone
                // left trigger spit out mouth
                if (_gamepad.left_trigger>Constants.armControlDeadzone) {
                    _armSubSystem.intakeReverse();
                    _armSubSystem.beltPower(_gamepad.left_trigger/2,_armSubSystem.lowerArmLimit());
                    continue;
                }

                // TODO set dead zone
                // suck into robot
                if (_gamepad.right_trigger>Constants.armControlDeadzone) {
                    _armSubSystem.intakeForward();
                    _armSubSystem.beltPower(-_gamepad.right_trigger/2,_armSubSystem.lowerArmLimit());
                    continue;
                }
            }




            _armSubSystem.beltPower(0.0,false);
            _armSubSystem.intakeStop();


        }

    }
}
