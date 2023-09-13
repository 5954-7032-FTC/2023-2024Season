package org.firstinspires.ftc.teamcode.commands.Mecanum;

import org.firstinspires.ftc.teamcode.commands.Command;
import org.firstinspires.ftc.teamcode.subsystems.SubSystem;
import org.firstinspires.ftc.teamcode.subsystems.DriveRobot;

public class Rotate implements Command {

    DriveRobot _drive;
    double _angle;

    public Rotate(DriveRobot drive, double angle) {
        this._drive = _drive;
        this._angle = _angle;
    }

    @Override
    public void Execute() {
        _drive.turn(_angle);
    }

    @Override
    public SubSystem getHardwareDevice() {
        return _drive;
    }
}
