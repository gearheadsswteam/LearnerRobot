package org.firstinspires.ftc.teamcode.command;
import android.util.Pair;
import java.util.Arrays;
import java.util.HashMap;
import java.util.HashSet;
import java.util.Map;
import java.util.Set;
import java.util.function.Function;
public class StateMachineBuilder<T extends Enum<T>> {
    private Set<T> states = new HashSet<>();
    private Map<Pair<T, T>, Function<Object[], Command>> transitions = new HashMap<>();
    private Scheduler scheduler;
    public StateMachineBuilder(CommandOpMode opMode) {
        scheduler = opMode.scheduler;
    }
    public StateMachineBuilder<T> addState(T... state) {
        states.addAll(Arrays.asList(state));
        return this;
    }
    public StateMachineBuilder<T> addTransition(T start, T end, Command command) {
        return addTransition(start, end, d -> command);
    }
    public StateMachineBuilder<T> addTransition(T start, T end, Function<Object[], Command> fn) {
        if (!states.contains(start) || !states.contains(end)) {
            throw new IllegalArgumentException("State does not exist");
        }
        transitions.put(new Pair<>(start, end), fn);
        return this;
    }
    public StateMachine<T> build(T state) {
        return new StateMachine<>(scheduler, state, states, transitions);
    }
}