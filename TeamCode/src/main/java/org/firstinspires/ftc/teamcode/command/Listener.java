package org.firstinspires.ftc.teamcode.command;
import java.util.function.BooleanSupplier;
public class Listener {
    private Command command;
    private BooleanSupplier condition;
    public Listener(BooleanSupplier condition, Command command) {
        this.condition = condition;
        this.command = command;
    }
    public boolean ready() {
        return condition.getAsBoolean();
    }
    public Command getCommand() {
        return command;
    }
}
