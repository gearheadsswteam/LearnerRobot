package org.firstinspires.ftc.teamcode.command;
import java.util.function.BooleanSupplier;
public class RisingEdgeDetector implements BooleanSupplier {
    private BooleanSupplier condition;
    private boolean last;
    public static Listener listen(BooleanSupplier condition, Command command) {
        return new Listener(new RisingEdgeDetector(condition), command);
    }
    public RisingEdgeDetector(BooleanSupplier condition) {
        this.condition = condition;
        last = false;
    }
    @Override
    public boolean getAsBoolean() {
        boolean temp = last;
        last = condition.getAsBoolean();
        return (last = condition.getAsBoolean()) && !temp;
    }
}
