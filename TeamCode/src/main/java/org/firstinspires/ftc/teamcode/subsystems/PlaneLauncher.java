package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.Servo;

public class PlaneLauncher implements SubSystem{
    Servo _launchServo;

    public PlaneLauncher(Servo launchServo) {
        _launchServo = launchServo;
    }

    public void launch() {
        _launchServo.setPosition(Servo.MIN_POSITION);
    }
    public void reset() {
        _launchServo.setPosition(Servo.MAX_POSITION);
    }
    public void set(double value) {
        _launchServo.setPosition(value);
    }
}
