package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.TouchSensor;

public class ArmSubSystem implements SubSystem {
    private CRServo[] _intake_servos;
    private DcMotor _lowerBeltMotor;
    private DcMotor _upperBeltMotor;
    private DcMotor[] _bendMotors;
    private TouchSensor _upperArmLimit;
    private TouchSensor _lowerArmLimit;

    public ArmSubSystem(CRServo[] intake_servos,
                        DcMotor lowerBeltMotor,
                        DcMotor upperBeltMotor,
                        DcMotor[] bendMotors,
                        TouchSensor upperArmLimit,
                        TouchSensor lowerArmLimit) {
        this._intake_servos = intake_servos;
        this._lowerBeltMotor = lowerBeltMotor;
        this._upperBeltMotor = upperBeltMotor;
        this._bendMotors = bendMotors;
        this._upperArmLimit = upperArmLimit;
        this._lowerArmLimit = lowerArmLimit;
        this.init();
    }

    private void init() {
        for (CRServo servo : _intake_servos) {
            servo.setDirection(DcMotorSimple.Direction.FORWARD);
        }
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
        for (CRServo servo: _intake_servos) {
            servo.setPower(0);
        }
    }

    public void intakeReverse() {
        for (CRServo servo: _intake_servos) {
            servo.setPower(-1);
        }
    }
    public void intakeForward() {
        for (CRServo servo: _intake_servos) {
            servo.setPower(1);
        }
    }

    private void lowerBeltPower(double power) {
        _lowerBeltMotor.setPower(power);
    }

    private void upperBeltPower(double power) {
        _upperBeltMotor.setPower(power);
    }

    public void beltPower(double power) {
        lowerBeltPower(power);
        upperBeltPower(power);
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

}
