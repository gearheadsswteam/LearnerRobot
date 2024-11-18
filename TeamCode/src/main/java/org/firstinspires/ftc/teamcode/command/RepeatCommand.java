package org.firstinspires.ftc.teamcode.command;
public class RepeatCommand extends Command {
    private Command repeat;
    private int i = 0;
    private int num;
    public RepeatCommand(Command repeat) {
        this(repeat, Integer.MAX_VALUE);
    }
    public RepeatCommand(Command repeat, int num) {
        this.repeat = repeat;
        this.num = num;
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
            i++;
            if (i < num) {
                repeat.init(time);
            }
        } else {
            repeat.run(time);
        }
    }
    @Override
    public void end(double time, boolean canceled) {
        if (canceled) {
            repeat.end(time, true);
        }
    }
    @Override
    public boolean done(double time) {
        return i >= num;
    }
}
