package org.firstinspires.ftc.teamcode.command;
import java.util.Arrays;
import java.util.HashSet;
import java.util.function.BiConsumer;
import java.util.function.DoubleConsumer;
import java.util.function.DoublePredicate;
public class FnCommand extends Command {
    private DoubleConsumer initFn;
    private DoubleConsumer runFn;
    private BiConsumer<Double, Boolean> endFn;
    private DoublePredicate doneFn;
    public static FnCommand once(DoubleConsumer initFn, Subsystem... systems) {
        return once(initFn, false, systems);
    }
    public static FnCommand once(DoubleConsumer initFn, boolean cancelable, Subsystem... systems) {
        return new FnCommand(initFn, t -> {}, (t, b) -> {}, t -> true, cancelable, systems);
    }
    public static FnCommand repeat(DoubleConsumer runFn, Subsystem... systems) {
        return repeat(runFn, false, systems);
    }
    public static FnCommand repeat(DoubleConsumer runFn, boolean cancelable, Subsystem... systems) {
        return new FnCommand(t -> {}, runFn, (t, b) -> {}, t -> false, cancelable, systems);
    }
    public static FnCommand until(DoublePredicate doneFn) {
        return until(doneFn, false);
    }
    public static FnCommand until(DoublePredicate doneFn, boolean cancelable) {
        return new FnCommand(t -> {}, t -> {}, (t, b) -> {}, doneFn, cancelable);
    }
    public FnCommand(DoubleConsumer initFn, DoubleConsumer runFn, BiConsumer<Double, Boolean> endFn, DoublePredicate doneFn, Subsystem... systems) {
        this(initFn, runFn, endFn, doneFn, false, systems);
    }
    public FnCommand(DoubleConsumer initFn, DoubleConsumer runFn, BiConsumer<Double, Boolean> endFn, DoublePredicate doneFn, boolean cancelable, Subsystem... systems) {
        subsystems.addAll(Arrays.asList(systems));
        this.cancelable = cancelable;
        this.initFn = initFn;
        this.runFn = runFn;
        this.endFn = endFn;
        this.doneFn = doneFn;
    }
    @Override
    public void init(double time) {
        initFn.accept(time);
    }
    @Override
    public void run(double time) {
        runFn.accept(time);
    }
    @Override
    public void end(double time, boolean canceled) {
        endFn.accept(time, canceled);
    }
    @Override
    public boolean done(double time) {
        return doneFn.test(time);
    }
}
