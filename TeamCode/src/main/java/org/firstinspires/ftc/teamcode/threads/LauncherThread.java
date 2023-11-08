package org.firstinspires.ftc.teamcode.threads;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.subsystems.PlaneLauncher;

public class LauncherThread extends RobotThread {
    PlaneLauncher _pl;
    Gamepad _gamepad;

    public LauncherThread(Servo launch, Gamepad gamepad) {
        this._pl = new PlaneLauncher(launch);
        this._gamepad = gamepad;
    }

    @Override
    public void run() {
        if (_gamepad.y) {
            _pl.launch();
            try {
                sleep(150);
            }
            catch (Exception e) {

            }
            _pl.reset();

        }
    }
}
