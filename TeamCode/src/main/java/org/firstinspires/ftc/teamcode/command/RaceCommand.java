package org.firstinspires.ftc.teamcode.command;
public class RaceCommand extends Command {
    private Command[] commands;
    public RaceCommand(Command... commands) {
        for (Command command : commands) {
            for (Subsystem subsystem : command.getSubsystems()) {
                if (subsystems.contains(subsystem)) {
                    throw new IllegalArgumentException("Subsystem used twice");
                }
                subsystems.add(subsystem);
            }
            cancelable = cancelable && command.cancelable;
        }
        this.commands = commands;
    }
    @Override
    public void init(double time) {
        for (Command command : commands) {
            command.init(time);
        }
    }
    @Override
    public void run(double time) {
        for (Command command : commands) {
            command.run(time);
        }
    }
    @Override
    public void end(double time, boolean canceled) {
        for (Command command : commands) {
            command.end(time, canceled);
        }
    }
    @Override
    public boolean done(double time) {
        for (Command command : commands) {
            if (command.done(time)) {
                return true;
            }
        }
        return false;
    }
}
