package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.teamcode.util.Constants;

public class ArmSubSystem implements SubSystem {
    private CRServo[] _intake_servos;
    private DcMotor _lowerBeltMotor;
    private DcMotor _upperBeltMotor;
    private DcMotor[] _bendMotors;
    private TouchSensor _upperArmLimit;
    private TouchSensor _lowerArmLimit;
    private Servo[] _pixelHold;

    public ArmSubSystem(CRServo[] intake_servos,
                        DcMotor lowerBeltMotor,
                        DcMotor upperBeltMotor,
                        DcMotor[] bendMotors,
                        TouchSensor upperArmLimit,
                        TouchSensor lowerArmLimit,
                        Servo [] pixelHold) {
        this._intake_servos = intake_servos;
        this._lowerBeltMotor = lowerBeltMotor;
        this._upperBeltMotor = upperBeltMotor;
        this._bendMotors = bendMotors;
        this._upperArmLimit = upperArmLimit;
        this._lowerArmLimit = lowerArmLimit;
        this._pixelHold = pixelHold;
        this.init();
    }

    private void init() {
        _intake_servos[0].setDirection(DcMotorSimple.Direction.REVERSE);
        _intake_servos[1].setDirection(DcMotorSimple.Direction.FORWARD);
        _lowerBeltMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        _lowerBeltMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        _upperBeltMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        _upperBeltMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        for (DcMotor motor: _bendMotors) {
            motor.setDirection(DcMotorSimple.Direction.FORWARD);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
    }

    public void intakeStop() {
        /*for (CRServo servo: _intake_servos) {
            servo.setPower(0);
        }
        */
        _intake_servos[0].setPower(0);
        _intake_servos[1].setPower(0);
    }

    public void intakeForward() {
        /*
        for (CRServo servo: _intake_servos) {
            servo.setPower(-0.5);
        }
        */
        _intake_servos[0].setPower(-0.5);
        _intake_servos[1].setPower(-0.5);
    }
    public void intakeReverse() {
        /*
        for (CRServo servo: _intake_servos) {
            servo.setPower(0.5);
        }
         */
        _intake_servos[0].setPower(0.5);
        _intake_servos[1].setPower(0.5);
    }

    private void lowerBeltPower(double power) {
        _lowerBeltMotor.setPower(power);
    }

    private void upperBeltPower(double power) {
        _upperBeltMotor.setPower(power);
    }

    public void beltPower(double power, boolean reverse) {
        lowerBeltPower(power);
        upperBeltPower( (reverse?-power:power) *0.8);
    }

    public void runBeltInches(double inches) {
        int counts = (int)(inches* Constants.BELT_COUNTS_PER_INCH);

        _lowerBeltMotor.setTargetPosition(_lowerBeltMotor.getCurrentPosition()+counts);
        _upperBeltMotor.setTargetPosition(_upperBeltMotor.getCurrentPosition()-counts);
        _lowerBeltMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        _upperBeltMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        _lowerBeltMotor.setPower(1.0);
        _upperBeltMotor.setPower(0.8);
        while (_lowerBeltMotor.isBusy()) {

        }
        _upperBeltMotor.setPower(0);
        _lowerBeltMotor.setPower(0);
        _upperBeltMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        _lowerBeltMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    public void moveArm(double power) {
        for (DcMotor motor : _bendMotors) {
            motor.setPower(power);
        }
    }

    public boolean upperArmLimit() {
        return _upperArmLimit.isPressed();
    }
    public boolean lowerArmLimit() {
        return _lowerArmLimit.isPressed();
    }

    public void raisePixelHold() {
        _pixelHold[0].setPosition(Servo.MAX_POSITION);
        _pixelHold[1].setPosition(Servo.MIN_POSITION);

    }

    public void lowerPixelHold() {
        _pixelHold[1].setPosition(Servo.MAX_POSITION);
        _pixelHold[0].setPosition(Servo.MIN_POSITION);
    }

}
