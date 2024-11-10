package org.firstinspires.ftc.teamcode.command;
import java.util.function.BooleanSupplier;
public class ConsecutiveDetector implements BooleanSupplier {
    private BooleanSupplier condition;
    private int num;
    private int total;
    public static Listener listen(BooleanSupplier condition, int num, Command command) {
        return new Listener(new ConsecutiveDetector(condition, num), command);
    }
    public ConsecutiveDetector(BooleanSupplier condition, int num) {
        this.condition = condition;
        this.num = num;
        total = 0;
    }
    @Override
    public boolean getAsBoolean() {
        total = condition.getAsBoolean() ? total + 1 : 0;
        return total >= num;
    }
}
