package org.firstinspires.ftc.teamcode.commands;

import org.firstinspires.ftc.teamcode.subsystems.SubSystem;

import java.util.ArrayList;

public class ParallelCommandGroup implements CommandGroup {
    protected ArrayList<Thread> commands;


    @Override
    public SubSystem getHardwareDevice() {
            return null;
    }

    @Override
    public void Execute() {
        for (Thread command : commands) {
            command.start();
        }
    }

    @Override
    public void addCommands(Command... commandlist) {
        ArrayList<SubSystem> hardware = new ArrayList<>();
        for (Command command:commandlist) {
            hardware.add(command.getHardwareDevice());
            commands.add(new CommandThread(command));
        }
    }
}
