package org.firstinspires.ftc.teamcode.commands;

public class CommandThread extends Thread {
    Command command;
    public CommandThread(Command command) {
        this.command = command;
    }

    @Override
    public void run() {
        command.Execute();
    }
}
