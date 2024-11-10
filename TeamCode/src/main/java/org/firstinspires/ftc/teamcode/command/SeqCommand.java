package org.firstinspires.ftc.teamcode.command;
public class SeqCommand extends Command {
    private Command[] commands;
    private int index;
    public SeqCommand(Command... commands) {
        for (Command command : commands) {
            subsystems.addAll(command.getSubsystems());
            cancelable = cancelable && command.cancelable;
        }
        this.commands = commands;
    }
    @Override
    public void init(double time) {
        index = 0;
        commands[0].init(time);
    }
    @Override
    public void run(double time) {
        if (index != commands.length - 1 && commands[index].done(time)) {
            commands[index].end(time, false);
            index++;
            commands[index].init(time);
        } else {
            commands[index].run(time);
        }
    }
    @Override
    public void end(double time, boolean canceled) {
        commands[index].end(time, canceled);
    }
    @Override
    public boolean done(double time) {
        return index == commands.length - 1 && commands[index].done(time);
    }
}
