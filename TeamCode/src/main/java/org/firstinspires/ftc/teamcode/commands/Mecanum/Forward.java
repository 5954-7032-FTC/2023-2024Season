package org.firstinspires.ftc.teamcode.commands.Mecanum;

import org.firstinspires.ftc.teamcode.subsystems.DriveRobot;
import org.firstinspires.ftc.teamcode.commands.Command;
import org.firstinspires.ftc.teamcode.subsystems.SubSystem;

public class Forward implements Command {
    double _distance;
    DriveRobot _drive;

    @Override
    public SubSystem getHardwareDevice() {
        return _drive;
    }

    public Forward(DriveRobot _drive, double _distance) {
        this._distance = _distance;
        this._drive = _drive;
    }

    @Override
    public void Execute() {
        _drive.driveForward(_distance);
    }
}
