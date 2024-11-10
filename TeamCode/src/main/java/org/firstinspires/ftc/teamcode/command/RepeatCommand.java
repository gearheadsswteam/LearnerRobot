package org.firstinspires.ftc.teamcode.command;
public class RepeatCommand extends Command {
    private Command repeat;
    public RepeatCommand(Command repeat) {
        this.repeat = repeat;
        subsystems.addAll(repeat.getSubsystems());
    }
    @Override
    public void init(double time) {
        repeat.init(time);
    }
    @Override
    public void run(double time) {
        if (repeat.done(time)) {
            repeat.end(time, false);
            repeat.init(time);
        } else {
            repeat.run(time);
        }
    }
    @Override
    public void end(double time, boolean canceled) {
        repeat.end(time, true);
    }
    @Override
    public boolean done(double time) {
        return false;
    }
}
