package org.firstinspires.ftc.teamcode.command;
import android.util.Pair;
import java.util.Map;
import java.util.Set;
import java.util.function.Function;
public class StateMachine<T extends Enum<T>> {
    private Scheduler scheduler;
    private T state;
    private Set<T> states;
    private Map<Pair<T, T>, Function<Object[], Command>> transitions;
    protected StateMachine(Scheduler scheduler, T state, Set<T> states, Map<Pair<T, T>, Function<Object[], Command>> transitions) {
        if (!states.contains(state)) {
            throw new IllegalArgumentException("State does not exist");
        }
        this.scheduler = scheduler;
        this.state = state;
        this.states = states;
        this.transitions = transitions;
    }
    public T state() {
        return state;
    }
    public boolean transition(T next, Object... params) {
        if (!transitions.containsKey(new Pair<>(state, next))) {
            throw new IllegalArgumentException("Transition does not exist");
        }
        if (scheduler.schedule(transitions.get(new Pair<>(state, next)).apply(params))) {
            state = next;
            return true;
        }
        return false;
    }
}
