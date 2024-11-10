package org.firstinspires.ftc.teamcode.command;
import java.util.function.BooleanSupplier;
public class FallingEdgeDetector implements BooleanSupplier {
    private BooleanSupplier condition;
    private boolean last;
    public static Listener listen(BooleanSupplier condition, Command command) {
        return new Listener(new FallingEdgeDetector(condition), command);
    }
    public FallingEdgeDetector(BooleanSupplier condition) {
        this.condition = condition;
        last = false;
    }
    @Override
    public boolean getAsBoolean() {
        boolean temp = last;
        last = condition.getAsBoolean();
        return !(last = condition.getAsBoolean()) && temp;
    }
}
