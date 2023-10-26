package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.Servo;

public class DroneRelease implements SubSystem {
    Servo _releaseServo;
    public DroneRelease(Servo releaseServo) {
        _releaseServo = releaseServo;
    }

    public void launchDrone() {
        _releaseServo.setPosition(0);
    }

    public void resetLauncher() {
        _releaseServo.setPosition(1);
    }
}
