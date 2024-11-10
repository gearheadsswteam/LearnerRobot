package org.firstinspires.ftc.teamcode.command;
import java.util.Arrays;
import java.util.function.BiConsumer;
import java.util.function.DoubleConsumer;
public class WaitCommand extends Command {
    private DoubleConsumer initFn;
    private BiConsumer<Double, Boolean> endFn;
    private double end;
    private double seconds;
    public WaitCommand(double seconds) {
        this(t -> {}, seconds);
    }
    public WaitCommand(DoubleConsumer initFn, double seconds, Subsystem... systems) {
        this(initFn, seconds, false, systems);
    }
    public WaitCommand(DoubleConsumer initFn, double seconds, boolean cancelable, Subsystem... systems) {
        subsystems.addAll(Arrays.asList(systems));
        this.initFn = initFn;
        this.endFn = (t, b) -> {};
        this.cancelable = cancelable;
        this.seconds = seconds;
    }
    public WaitCommand(double seconds, BiConsumer<Double, Boolean> endFn, Subsystem... systems) {
        this(seconds, endFn, false, systems);
    }
    public WaitCommand(double seconds, BiConsumer<Double, Boolean> endFn, boolean cancelable, Subsystem... systems) {
        subsystems.addAll(Arrays.asList(systems));
        this.initFn = t -> {};
        this.endFn = endFn;
        this.cancelable = cancelable;
        this.seconds = seconds;
    }
    @Override
    public void init(double time) {
        end = time + seconds;
        initFn.accept(time);
    }
    @Override
    public void run(double time) {}
    @Override
    public void end(double time, boolean canceled) {
        endFn.accept(time, canceled);
    }
    @Override
    public boolean done(double time) {
        return time > end;
    }
}