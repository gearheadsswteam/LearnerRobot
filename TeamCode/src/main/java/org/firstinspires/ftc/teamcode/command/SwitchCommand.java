package org.firstinspires.ftc.teamcode.command;
import android.util.Pair;
import java.util.HashMap;
import java.util.Map;
import java.util.function.Supplier;
public class SwitchCommand<T> extends Command {
    private Command command;
    private Map<T, Command> map;
    private Supplier<T> decide;
    public SwitchCommand(Supplier<T> decide, Pair<T, Command>... pairs) {
        this.decide = decide;
        map = new HashMap<>();
        for (Pair<T, Command> pair : pairs) {
            subsystems.addAll(pair.second.subsystems);
            map.put(pair.first, pair.second);
        }
    }
    @Override
    public void init(double time) {
        command = map.get(decide.get());
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
