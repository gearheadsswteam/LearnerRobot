package org.firstinspires.ftc.teamcode.command;
import java.util.function.BooleanSupplier;
public class BoolCommand extends Command {
    private Command command;
    private Command onTrue;
    private Command onFalse;
    private BooleanSupplier decide;
    public BoolCommand(BooleanSupplier decide, Command onTrue, Command onFalse) {
        this.decide = decide;
        this.onTrue = onTrue;
        this.onFalse = onFalse;
        subsystems.addAll(onTrue.subsystems);
        subsystems.addAll(onFalse.subsystems);
    }
    @Override
    public void init(double time) {
        command = decide.getAsBoolean() ? onTrue : onFalse;
        command.init(time);
    }
    @Override
    public void run(double time) {
        command.run(time);
    }
    @Override
    public void end(double time, boolean canceled) {
        command.end(time, canceled);
    }
    @Override
    public boolean done(double time) {
        return command.done(time);
    }
}
