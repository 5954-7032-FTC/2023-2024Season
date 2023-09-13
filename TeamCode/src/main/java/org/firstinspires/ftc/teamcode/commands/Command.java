package org.firstinspires.ftc.teamcode.commands;

import org.firstinspires.ftc.teamcode.subsystems.SubSystem;

public interface Command {
    void Execute();
    SubSystem getHardwareDevice();
}
