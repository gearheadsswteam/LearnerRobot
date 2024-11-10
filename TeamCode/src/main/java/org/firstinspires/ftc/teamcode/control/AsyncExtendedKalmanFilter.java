package org.firstinspires.ftc.teamcode.control;
import org.ejml.simple.SimpleMatrix;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Comparator;
import java.util.Iterator;
import java.util.LinkedList;
import java.util.List;
import java.util.concurrent.LinkedBlockingQueue;
public abstract class AsyncExtendedKalmanFilter {
    private double window;
    protected LinkedList<KalmanState> states = new LinkedList<>();
    private List<LinkedBlockingQueue<KalmanEstimate>> sensors = new ArrayList<>();
    public AsyncExtendedKalmanFilter(double window) {
        this.window = window;
    }
    public void addSensors(LinkedBlockingQueue<KalmanEstimate>... sensor) {
        sensors.addAll(Arrays.asList(sensor));
    }
    public void run(double time) {
        states.getLast().u = measure(time);
        states.offer(new KalmanState(null, null, null, time));
        while (states.getFirst().time < time - window && states.size() > 2) {
            states.removeFirst();
        }
        List<KalmanEstimate> ests = new ArrayList<>();
        for (LinkedBlockingQueue<KalmanEstimate> queue : sensors) {
            while (!queue.isEmpty()) {
                ests.add(queue.poll());
            }
        }
        ests.sort(Comparator.comparingDouble(a -> a.time));
        int ind = 0;
        Iterator<KalmanState> it = states.iterator();
        KalmanState curr = it.next();
        for (int i = 0; i < states.size() - 1; i++) {
            KalmanState next = it.next();
            while (ests.get(ind).time < next.time) {
                if (ests.get(ind).time >= time - window) {
                    next.ests.add(ests.get(ind));
                }
                ind++;
            }
            if (ind != 0 || i == states.size() - 2) {
                predict(curr, next);
                correct(next);
            }
            curr = next;
        }
    }
    public SimpleMatrix state() {
        return states.getLast().x;
    }
    public void set(SimpleMatrix x, SimpleMatrix P) {
        states = new LinkedList<>(List.of(new KalmanState(x, null, P, states.getLast().time)));
    }
    public void set(SimpleMatrix x, SimpleMatrix P, double time) {
        states = new LinkedList<>(List.of(new KalmanState(x, null, P, time)));
    }
    private void predict(KalmanState in, KalmanState out) {
        SimpleMatrix F = F(in.x, in.u);
        out.x = f(in.x, in.u);
        out.P = F.mult(in.P).mult(F.transpose()).plus(Q(in.x));
    }
    private void correct(KalmanState state) {
        for (KalmanEstimate est : state.ests) {
            SimpleMatrix K = state.P.mult(est.H.transpose()).mult(est.H.mult(state.P).mult(est.H.transpose()).plus(est.R).invert());
            SimpleMatrix M = SimpleMatrix.identity(state.x.getNumRows()).minus(K.mult(est.H));
            state.x = M.mult(state.x).plus(K.mult(est.z));
            state.P = M.mult(state.P);
        }
    }
    protected abstract SimpleMatrix measure(double time);
    protected abstract SimpleMatrix f(SimpleMatrix x, SimpleMatrix u);
    protected abstract SimpleMatrix F(SimpleMatrix x, SimpleMatrix u);
    protected abstract SimpleMatrix Q(SimpleMatrix x);
}