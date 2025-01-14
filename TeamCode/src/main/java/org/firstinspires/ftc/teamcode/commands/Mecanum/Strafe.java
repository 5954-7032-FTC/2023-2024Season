package org.firstinspires.ftc.teamcode.commands.Mecanum;

import org.firstinspires.ftc.teamcode.commands.Command;
import org.firstinspires.ftc.teamcode.subsystems.SubSystem;
import org.firstinspires.ftc.teamcode.subsystems.DriveRobot;

public class Strafe implements Command {
    protected DriveRobot _drive;
    protected double _distance;

    public Strafe(DriveRobot _drive, double distance) {
        this._drive = _drive;
        this._distance = distance;
    }

    @Override
    public void Execute() {
        if (_distance >0 )
            _drive.driveRight(_distance);
        else
            _drive.driveLeft(-_distance);
    }

    @Override
    public SubSystem getHardwareDevice() {
        return _drive;
    }
}
