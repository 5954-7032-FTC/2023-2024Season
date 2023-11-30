package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

public class ArmSubSystem implements SubSystem {
    private final CRServo[] _intake_servos;
    private final DcMotor _lowerBeltMotor;
    private final DcMotor _upperBeltMotor;
    private final DcMotor[] _bendMotors;
    private final TouchSensor _upperArmLimit;
    private final TouchSensor _lowerArmLimit;
    private final Servo[] _pixelHold;

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

        _intake_servos[0].setPower(0);
        _intake_servos[1].setPower(0);
    }

    public void intakeForward() {

        _intake_servos[0].setPower(-0.5);
        _intake_servos[1].setPower(-0.5);
    }
    public void intakeReverse() {

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
        upperBeltPower( -power*0.8);//(reverse?-power:power) *0.8);
    }

    public void runBeltMillis(boolean forward, long millis){

        _lowerBeltMotor.setPower(forward?1.0:-1.0);
        _upperBeltMotor.setPower(forward?-0.8:0.8);
        try {
            Thread.sleep(millis);
        }
        catch ( InterruptedException ignored) {
        }
        _upperBeltMotor.setPower(0);
        _lowerBeltMotor.setPower(0);
    }

    public void moveArmMillis(boolean up,long millis) {
        moveArm(up?-1:1);
        try {
            Thread.sleep(millis);
        }
        catch (InterruptedException ignored) {
        }
        moveArm(0);
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
