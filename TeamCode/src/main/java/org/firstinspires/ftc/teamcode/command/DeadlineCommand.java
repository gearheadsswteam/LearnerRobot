package org.firstinspires.ftc.teamcode.command;
public class DeadlineCommand extends Command {
    private Command primary;
    private Command[] commands;
    private boolean[] dones;
    public DeadlineCommand(Command primary, Command... commands) {
        subsystems.addAll(primary.getSubsystems());
        cancelable = cancelable && primary.isCancelable();
        for (Command command : commands) {
            for (Subsystem subsystem : command.getSubsystems()) {
                if (subsystems.contains(subsystem)) {
                    throw new IllegalArgumentException("Subsystem used twice");
                }
                subsystems.add(subsystem);
                cancelable = cancelable && command.isCancelable();
            }
        }
        this.primary = primary;
        this.commands = commands;
    }
    @Override
    public void init(double time) {
        dones = new boolean[commands.length];
        primary.init(time);
        for (Command command : commands) {
            command.init(time);
        }
    }
    @Override
    public void run(double time) {
        primary.run(time);
        for (int i = 0; i < commands.length; i++) {
            if (!dones[i]) {
                if (commands[i].done(time)) {
                    dones[i] = true;
                    commands[i].end(time, false);
                } else {
                    commands[i].run(time);
                }
            }
        }
    }
    @Override
    public void end(double time, boolean canceled) {
        if (canceled) {
            primary.end(time, true);
            for (int i = 0; i < commands.length; i++) {
                if (!dones[i]) {
                    commands[i].end(time, true);
                }
            }
        }
        primary.end(time, false);
        for (Command command : commands) {
            if (command.done(time)) {
                command.end(time, false);
            }
        }
    }
    @Override
    public boolean done(double time) {
        return primary.done(time);
    }
}
